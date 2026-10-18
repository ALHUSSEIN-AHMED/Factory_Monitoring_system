"""
IoT Predictive Machine Monitoring & Protection  -  Raspberry Pi Pico W (MicroPython)
Sensors : DHT22 (temp/humidity), MPU6050 (vibration), potentiometer (stands in for ACS712 current sensor)
Outputs : relay (machine power), RGB status LED, buzzer, "MACHINE" LED
Cloud   : MQTT -> HiveMQ public broker -> Node-RED (dashboard, Telegram, e-mail, logging)

Safety logic (alarms, prediction, idle shutdown, overload stop) runs LOCALLY on the Pico,
so the machine stays protected even if WiFi / MQTT is down.
"""
import network
import time
import math
import json
import struct
from machine import Pin, I2C, ADC, PWM
import dht
from umqtt_simple import MQTTClient

# ======================= CONFIG =======================
# ======================= CONFIG =======================
MACHINE_ID = "M01"

WIFI_SSID = "Wokwi-GUEST"
WIFI_PASS = ""

# HiveMQ Cloud credentials
BROKER = ""
MQTT_PORT = 8883
MQTT_USER = ""
MQTT_PASSWORD = ""

# MQTT topics
TOPIC_ROOT = "pdm_demo_x7k2/" + MACHINE_ID

T_TELEMETRY = TOPIC_ROOT + "/telemetry"
T_EVENT = TOPIC_ROOT + "/event"
T_STATUS = TOPIC_ROOT + "/status"
T_CMD = TOPIC_ROOT + "/cmd"

# Thresholds
T_WARN, T_CRIT = 50.0, 65.0            # degC
RATE_PRED = 5.0                        # degC/min -> predictive warning
T_PRED_MIN = 35.0                      # ignore rate below this temperature
RATE_WINDOW_MS = 30000                 # window used to compute temperature slope
V_WARN, V_CRIT = 0.30, 0.80            # g (RMS deviation from 1 g)
CURRENT_FULL_SCALE = 10.0              # A represented by pot at 100 %
I_IDLE = 0.5                           # A  below this = no effective load
I_WARN, I_CRIT = 6.5, 8.5              # A
I_CRIT_HOLD_MS = 3000                  # overload must last 3 s (ignore start-up inrush)
IDLE_SECONDS = 120                     # spec = 2 min  (set 20 to test faster)
IDLE_VARIATION = 0.3                   # A, change that resets the idle timer

MQ2_WARN = 30000                       
MQ2_CRIT = 50000

# ======================= HARDWARE =======================
wifi_led = Pin("LED", Pin.OUT)         # onboard LED on = MQTT connected
dht_sensor = dht.DHT22(Pin(15))
i2c = I2C(0, sda=Pin(4), scl=Pin(5), freq=400000)
adc = ADC(26)
mq2_AO = machine.ADC(27)
relay = Pin(16, Pin.OUT, value=0)
button = Pin(14, Pin.IN, Pin.PULL_UP)
buzzer = PWM(Pin(13)); buzzer.freq(2000); buzzer.duty_u16(0)
led_r, led_g, led_b = PWM(Pin(10)), PWM(Pin(11)), PWM(Pin(12))
for _p in (led_r, led_g, led_b):
    _p.freq(1000)

MPU = 0x68

def mpu_init():
    i2c.writeto_mem(MPU, 0x6B, b"\x00")   # wake up

def read_vibration():
    """RMS deviation of |a| from 1 g over a short burst (orientation independent)."""
    s, n = 0.0, 10
    for _ in range(n):
        d = i2c.readfrom_mem(MPU, 0x3B, 6)
        ax, ay, az = struct.unpack(">hhh", d)
        m = math.sqrt(ax * ax + ay * ay + az * az) / 16384.0
        s += (m - 1.0) ** 2
        time.sleep_ms(5)
    return math.sqrt(s / n)

def read_current():
    """Pot 0-3.3 V -> 0-10 A.
    Real ACS712-20A: amps = (V_adc * divider_ratio - 2.5) / 0.100  (use RMS over many samples)."""
    raw = 0
    for _ in range(8):
        raw += adc.read_u16()
    v = (raw / 8) * 3.3 / 65535
    return v / 3.3 * CURRENT_FULL_SCALE


def read_gas():
    """Read MQ-2 analog output (AO) via ADC."""
    raw = 0
    for _ in range(8):
        raw += mq2_AO.read_u16()
    return raw / 8


# ======================= STATE =======================
LEVELS = ("NORMAL", "WARNING", "PREDICTIVE", "CRITICAL")
HEALTH = ("Healthy", "Warning", "At Risk", "Critical")

machine_on = False
mstate = "STOPPED_MANUAL"              # RUNNING / STOPPED_MANUAL / STOPPED_IDLE / STOPPED_PROTECTION
temp = hum = None
vib = 0.0
amps = 0.0

gas = 0.0

rate = 0.0
level = 0
temp_hist = []
active = {}
idle_since = None
idle_ref = 0.0
crit_since = None
client = None
pending = []                           # events buffered while offline
wlan = network.WLAN(network.STA_IF)

# ======================= MQTT =======================
def pub(topic, payload, retain=False):
    global client
    if client is None:
        return False
    try:
        client.publish(topic, payload, retain)
        return True
    except Exception as e:
        print("publish failed:", e)
        client = None
        wifi_led.off()
        return False

def send_event(etype, severity, msg, action=""):
    ev = {"machine": MACHINE_ID, "uptime_s": time.ticks_ms() // 1000, "type": etype,
          "severity": severity, "message": msg, "action": action,
          "temp": temp, "vib_g": round(vib, 3), "current_a": round(amps, 2), "state": mstate ,"Gas":gas}
    print("EVENT:", severity, etype, "-", msg)
    data = json.dumps(ev)
    if not pub(T_EVENT, data):
        if len(pending) < 20:
            pending.append(data)

def on_msg(topic, msg):
    cmd = msg.decode().strip().upper()
    print("CMD:", cmd)
    if cmd == "START":
        start_machine("remote command")
    elif cmd == "STOP":
        stop_machine("STOPPED_MANUAL", "MANUAL_STOP", "INFO", "Machine stopped by remote command")

def wifi_connect():
    wlan.active(True)
    if wlan.isconnected():
        return True
    wlan.connect(WIFI_SSID, WIFI_PASS)
    for _ in range(40):
        if wlan.isconnected():
            print("WiFi OK", wlan.ifconfig()[0])
            return True
        time.sleep_ms(250)
    return False


def mqtt_connect():
    global client

    try:
        print("Connecting to HiveMQ...")
        print("Broker:", BROKER)
        print("Port:", MQTT_PORT)

        c = MQTTClient(
            client_id="pico_" + MACHINE_ID,
            server=BROKER,
            port=MQTT_PORT,
            user=MQTT_USER,
            password=MQTT_PASSWORD,
            keepalive=60,
            ssl=True,
            ssl_params={"server_hostname": BROKER}
        )

        c.set_last_will(T_STATUS, b"offline", retain=True)
        c.set_callback(on_msg)

        c.connect()
        print("MQTT connection successful!")

        c.subscribe(T_CMD)
        print("Subscribed to:", T_CMD)

        client = c
        wifi_led.on()

        pub(T_STATUS, b"online", True)
        print("Status published.")

        while pending:
            if not pub(T_EVENT, pending[0]):
                break
            pending.pop(0)

    except Exception as e:
        print("MQTT connection failed:", repr(e))
        client = None
        wifi_led.off()


# ======================= MACHINE CONTROL =======================
def start_machine(source):
    global machine_on, mstate, idle_since, crit_since
    if machine_on:
        return
    if level == 3:
        send_event("RESTART_DENIED", "WARNING", "Restart refused: critical condition still present",
                   "Clear the fault, wait for readings to normalise, then restart")
        return
    machine_on, mstate = True, "RUNNING"
    idle_since = crit_since = None
    relay.value(1)
    send_event("MACHINE_STARTED", "INFO", "Machine started (" + source + ")")

def stop_machine(state, etype, severity, msg, action=""):
    global machine_on, mstate
    if not machine_on:
        return
    machine_on, mstate = False, state
    relay.value(0)
    send_event(etype, severity, msg, action)

# ======================= ANALYSIS =======================
def update_rate(now):
    global rate
    temp_hist.append((now, temp))
    while temp_hist and time.ticks_diff(now, temp_hist[0][0]) > RATE_WINDOW_MS:
        temp_hist.pop(0)
    span = time.ticks_diff(temp_hist[-1][0], temp_hist[0][0])
    rate = (temp_hist[-1][1] - temp_hist[0][1]) / (span / 60000.0) if span >= 8000 else 0.0

def evaluate(now):
    """Return list of (type, level, message, recommended action)."""
    global idle_since, idle_ref, crit_since
    issues = []
    if temp is not None:
        if temp >= T_CRIT:
            issues.append(("HIGH_TEMPERATURE", 3, "Temperature %.1fC above critical limit %.0fC" % (temp, T_CRIT),
                           "Stop machine, check cooling, lubrication and bearings"))
        elif temp >= T_WARN:
            issues.append(("HIGH_TEMPERATURE", 1, "Temperature %.1fC above warning limit %.0fC" % (temp, T_WARN),
                           "Inspect cooling and ventilation"))
        if rate >= RATE_PRED and temp >= T_PRED_MIN and temp < T_CRIT:
            eta = (T_CRIT - temp) / rate
            issues.append(("TEMP_RISING_FAST", 2,
                           "Predictive warning: temperature rising %.1f C/min, critical in ~%.1f min" % (rate, eta),
                           "Inspect the machine before it reaches the critical level"))


    if gas >= MQ2_CRIT:
        issues.append(("GAS_CRITICAL", 3, "Gas/Smoke level %.0f above critical limit %d" % (gas, MQ2_CRIT),
                       "Stop machine immediately, check for gas leak or smoke source"))
    elif gas >= MQ2_WARN:
        issues.append(("GAS_WARNING", 1, "Gas/Smoke level %.0f above warning limit %d" % (gas, MQ2_WARN),
                       "Inspect area for potential gas or smoke buildup"))



    if vib >= V_CRIT:
        issues.append(("HIGH_VIBRATION", 3, "Vibration %.2fg above critical limit %.2fg" % (vib, V_CRIT),
                       "Stop machine, check balance, mounting and bearings"))
    elif vib >= V_WARN:
        issues.append(("HIGH_VIBRATION", 1, "Vibration %.2fg above warning limit %.2fg" % (vib, V_WARN),
                       "Schedule bearing / alignment inspection"))
    if machine_on:
        if amps >= I_CRIT:
            if crit_since is None:
                crit_since = now
            lvl = 3 if time.ticks_diff(now, crit_since) >= I_CRIT_HOLD_MS else 1
            issues.append(("OVERLOAD", lvl, "Current %.1fA above overload limit %.1fA" % (amps, I_CRIT),
                           "Reduce load or check for jam / mechanical blockage"))
        else:
            crit_since = None
            if amps >= I_WARN:
                issues.append(("OVERLOAD", 1, "Current %.1fA above %.1fA" % (amps, I_WARN),
                               "Monitor load, possible abnormal load"))
        # idle detection: ON + low current + low variation for IDLE_SECONDS
        if amps < I_IDLE:
            if idle_since is None or abs(amps - idle_ref) > IDLE_VARIATION:
                idle_since, idle_ref = now, amps
        else:
            idle_since = None
    else:
        idle_since = crit_since = None
    return issues

def process(now):
    global level, active
    issues = evaluate(now)
    cur = {}
    for typ, lvl, msg, act in issues:
        cur[typ] = lvl
        if active.get(typ, 0) < lvl:
            send_event(typ, LEVELS[lvl], msg, act)
    if active and not cur:
        send_event("RECOVERY", "NORMAL", "All readings back to normal", "Restart machine if it was stopped")
    active = cur
    level = max(cur.values()) if cur else 0

    # automatic protection
    if level == 3 and machine_on:
        worst = [i for i in issues if i[1] == 3][0]
        stop_machine("STOPPED_PROTECTION", "AUTO_SHUTDOWN", "CRITICAL",
                     "Machine stopped automatically: " + worst[2], "Inspect machine before restarting")
    # automatic idle shutdown
    if machine_on and idle_since is not None and time.ticks_diff(now, idle_since) >= IDLE_SECONDS * 1000:
        stop_machine("STOPPED_IDLE", "IDLE_SHUTDOWN", "INFO",
                     "Machine idle for %d s, switched off to save energy" % IDLE_SECONDS,
                     "Restart when production resumes")

# ======================= OUTPUTS =======================
def set_rgb(r, g, b):
    led_r.duty_u16(r * 257); led_g.duty_u16(g * 257); led_b.duty_u16(b * 257)

def update_outputs(now):
    if level == 3:
        set_rgb(255, 0, 0)
    elif level == 2:
        set_rgb(255, 70, 0)       # orange
    elif level == 1:
        set_rgb(255, 200, 0)      # yellow
    elif machine_on:
        set_rgb(0, 255, 0)
    else:
        set_rgb(0, 0, 255)        # healthy but machine OFF
    if level == 3:
        on = (now // 100) % 2 == 0
    elif level == 2:
        on = (now % 2000) < 150
    elif level == 1:
        on = (now % 5000) < 100
    else:
        on = False
    buzzer.duty_u16(20000 if on else 0)

def publish_telemetry(now):
    idle_s = time.ticks_diff(now, idle_since) // 1000 if idle_since is not None else 0
    d = {"machine": MACHINE_ID, "uptime_s": now // 1000, "temp": temp, "hum": hum,
         "temp_rate": round(rate, 2), "vib_g": round(vib, 3), "current_a": round(amps, 2),
         
         "gas": round(gas, 1),

         "machine_on": machine_on, "state": mstate, "level": LEVELS[level],
         "health": HEALTH[level], "idle_s": idle_s}
    data = json.dumps(d)
    print(data)
    pub(T_TELEMETRY, data)

# ======================= MAIN =======================
mpu_init()
set_rgb(0, 0, 255)
wifi_connect()
mqtt_connect()
start_machine("boot")

t_cur = t_vib = t_dht = t_pub = t_net = t_gas = time.ticks_ms()
btn_prev = 1

while True:
    now = time.ticks_ms()

    if time.ticks_diff(now, t_cur) >= 250:
        t_cur = now
        amps = read_current()


    if time.ticks_diff(now, t_gas) >= 500:
        t_gas = now
        try:
            gas = read_gas()
        except OSError:
            pass


    if time.ticks_diff(now, t_vib) >= 500:
        t_vib = now
        try:
            vib = read_vibration()
        except OSError:
            pass

    if time.ticks_diff(now, t_dht) >= 2000:       # DHT22 max rate = 0.5 Hz
        t_dht = now
        try:
            dht_sensor.measure()
            temp, hum = dht_sensor.temperature(), dht_sensor.humidity()
            update_rate(now)
        except OSError:
            pass

    process(now)
    update_outputs(now)

    if time.ticks_diff(now, t_pub) >= 2000:
        t_pub = now
        publish_telemetry(now)

    # button: toggle machine ON/OFF
    b = button.value()
    if btn_prev == 1 and b == 0:
        if machine_on:
            stop_machine("STOPPED_MANUAL", "MANUAL_STOP", "INFO", "Machine stopped by operator button")
        else:
            start_machine("button")
    btn_prev = b

    # incoming MQTT commands
    if client:
        try:
            client.check_msg()
        except Exception:
            client = None
            wifi_led.off()

    # reconnect (every 15 s) - protection keeps working while offline
    if client is None and time.ticks_diff(now, t_net) >= 15000:
        t_net = now
        if wifi_connect():
            mqtt_connect()

    time.sleep_ms(50)