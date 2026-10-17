#!/usr/bin/env python3
"""
================================================================================
IoT Graduation Project: Overcurrent Protection & Edge-AI Predictive Maintenance
(Refactored - bench-friendly, fault-tolerant, fast protection loop)
================================================================================
Hardware Supported (Auto-detected, fault-tolerant if disconnected):
  - Raspberry Pi 4 Model B
  - INA219 (I2C)        : Current, Voltage & Power Monitoring (Overcurrent Trip)
  - MPU6050 (I2C)       : 3-Axis Vibration / Acceleration & Gyroscope
  - PN532 NFC (I2C)     : Tap-to-Toggle Motor ON/OFF (Supports 2 Tags + Open Mode)
  - I2C LCD (16x2/20x4) : Real-time Local Status & Alert Logging
  - DHT22 (GPIO)        : Temperature & Humidity Monitoring
  - MQ5 (GPIO Digital)  : Gas / Smoke Leak Detection
  - Relay + DC Motor    : Actuator with Hardware/Software Safety Cut-off

Cloud, Edge AI & Integrations:
  - Edge Impulse Linux SDK (.eim) + NumPy FFT Spectral Feature Extraction
  - HiveMQ Cloud (MQTT over TLS) -> Feeds Node-RED Dashboard & Controls
  - ThingsBoard Telemetry (Direct MQTT)
  - Telegram Bot API (Instant Fault & Status Alerts)

Relay semantics:  relay ON = motor RUNNING.  Any safety trip turns relay OFF.

Bench mode (no NFC reader connected):
  - Config.AUTO_START_MOTOR       -> motor starts at boot
  - Config.AUTO_CLEAR_GAS_FAULT   -> motor resumes when gas clears
  - Terminal commands: on | off | reset | status | quit

Dependencies Installation (Run in Terminal):
  sudo apt-get update && sudo apt-get install -y python3-pip python3-smbus i2c-tools libgpiod2
  pip3 install smbus2 RPi.GPIO adafruit-blinka adafruit-circuitpython-ina219 \
               adafruit-circuitpython-mpu6050 adafruit-circuitpython-dht \
               adafruit-circuitpython-pn532 paho-mqtt requests numpy \
               edge_impulse_linux
================================================================================
"""

import os
import sys
import time
import json
import ssl
import math
import queue
import logging
import threading
from datetime import datetime

# Optional / Fault-Tolerant Third-Party Imports
try:
    import numpy as np
    NUMPY_AVAILABLE = True
except ImportError:
    NUMPY_AVAILABLE = False

try:
    import requests
    REQUESTS_AVAILABLE = True
except ImportError:
    REQUESTS_AVAILABLE = False

try:
    import paho.mqtt.client as mqtt
    MQTT_AVAILABLE = True
except ImportError:
    MQTT_AVAILABLE = False

try:
    import smbus2
    SMBUS_AVAILABLE = True
except ImportError:
    SMBUS_AVAILABLE = False

try:
    import RPi.GPIO as GPIO
    GPIO_AVAILABLE = True
except ImportError:
    GPIO_AVAILABLE = False

try:
    import board
    import busio
    BLINKA_AVAILABLE = True
except ImportError:
    BLINKA_AVAILABLE = False

try:
    import adafruit_ina219
    INA219_LIB_AVAILABLE = True
except ImportError:
    INA219_LIB_AVAILABLE = False

try:
    import adafruit_mpu6050
    MPU6050_LIB_AVAILABLE = True
except ImportError:
    MPU6050_LIB_AVAILABLE = False

try:
    import adafruit_dht
    DHT_LIB_AVAILABLE = True
except ImportError:
    DHT_LIB_AVAILABLE = False

try:
    from adafruit_pn532.i2c import PN532_I2C
    PN532_LIB_AVAILABLE = True
except ImportError:
    PN532_LIB_AVAILABLE = False

try:
    from edge_impulse_linux.runner import ImpulseRunner
    EDGE_IMPULSE_AVAILABLE = True
except ImportError:
    EDGE_IMPULSE_AVAILABLE = False


# ==============================================================================
# 1. CONFIGURATION & THRESHOLDS
# ==============================================================================
class Config:
    # --- Bench / Deployment Switches ---
    AUTO_START_MOTOR = True          # Bench mode: start motor at boot (no NFC needed)
    AUTO_CLEAR_GAS_FAULT = True      # Bench mode: resume motor automatically when gas clears
    FAILSAFE_ON_INA_LOSS = False     # Production: trip if INA219 drops out while motor runs

    # --- GPIO Pin Definitions (BCM Numbering) ---
    PIN_DHT22 = 4             # GPIO 4  (Pin 7)
    PIN_MQ5_DO = 17           # GPIO 17 (Pin 11) - Digital Output from MQ5
    PIN_RELAY = 27            # GPIO 27 (Pin 13) - Relay Control Pin
    RELAY_ACTIVE_LOW = False  # False = HIGH turns Relay ON (matches your working setup). True = open-drain active-low
    MQ5_ACTIVE_LOW = True     # True if MQ5 DO pin goes LOW when gas is detected

    # --- MQ5 Behaviour ---
    MQ5_WARMUP_SEC = 3.0      # Ignore gas readings while the heater warms up (set 0 to disable; use ~20+ in real use)
    GAS_DEBOUNCE_READS = 3    # Consecutive positive reads (~20 ms apart) needed to confirm gas

    # --- I2C Addresses ---
    I2C_BUS_NUM = 1
    ADDR_LCD_PRIMARY = 0x27
    ADDR_LCD_SECONDARY = 0x3F
    ADDR_INA219 = 0x40
    ADDR_MPU6050 = 0x68
    ADDR_PN532 = 0x24
    LCD_COLS = 16
    LCD_ROWS = 2

    # --- NFC Tag Configuration ---
    # Add your 2 NFC tag hex UIDs here (e.g., ["04a1b2c3", "e2801122"]).
    # If left empty [], ANY scanned NFC tag will toggle ON/OFF and print its UID to terminal.
    AUTHORIZED_NFC_UIDS = [
        "63CB6D34",  # Tag 1
    ]
    NFC_DEBOUNCE_SECONDS = 2.0  # Minimum time between NFC taps to prevent rapid toggling
    NFC_REMOVE_GRACE_SEC = 0.8  # Tag must be absent this long before the next tap counts (stops re-toggling while a tag sits on the reader)

    # --- Protection & Predictive Maintenance Thresholds ---
    OVERCURRENT_LIMIT_A = 3.277          # Amperes: Trips relay if exceeded
    OVERCURRENT_CONFIRM_SAMPLES = 2     # Consecutive samples above limit before tripping (noise filter)
    START_INRUSH_GRACE_SEC = 0.5        # Ignore overcurrent briefly after motor start (inrush)
    WARNING_CURRENT_A = 1.30            # Amperes: Predictive warning threshold
    UNDERVOLTAGE_LIMIT_V = 4.50         # Volts (adjust to 10.5V for 12V motor supply)
    MAX_TEMP_C = 60.0                   # Celsius: Thermal cut-off threshold
    VIBRATION_RMS_LIMIT = 14.5          # m/s^2: Abnormal vibration threshold
    FFT_HARMONIC_ENERGY_LIMIT = 45.0    # Spectral energy threshold (vibration signal)
    CURRENT_AC_RMS_LIMIT_A = 0.35       # Amperes: abnormal current ripple (used when no MPU6050)
    CURRENT_FFT_ENERGY_LIMIT = 10.0     # Spectral energy threshold (current signal) - tune on real data

    # --- FFT & Edge Impulse Configuration ---
    EI_MODEL_PATH = "./modelfile.eim"   # Path to Edge Impulse Linux model (.eim)
    FFT_WINDOW_SIZE = 64                # Must be power of 2 (32, 64, 128)
    FFT_SAMPLE_RATE_HZ = 100            # Target sampling rate (real rate limited by I2C speed)
    EI_ANOMALY_THRESHOLD = 0.65         # Edge Impulse anomaly / fault probability threshold

    # --- HiveMQ Cloud (Used by Node-RED) ---
    HIVEMQ_ENABLED = True
    HIVEMQ_HOST = "83465edc64434586bfc5f610174682e6.s1.eu.hivemq.cloud"
    HIVEMQ_PORT = 8883
    HIVEMQ_USER = "777"
    HIVEMQ_PASS = "12345678"
    HIVEMQ_TOPIC_TELEMETRY = "gradproject/motor/telemetry"
    HIVEMQ_TOPIC_ALERTS = "gradproject/motor/alerts"
    HIVEMQ_TOPIC_COMMAND = "gradproject/motor/command"  # {"motor": "ON"|"OFF"|"RESET"}

    # --- ThingsBoard Cloud / Local Server ---
    THINGSBOARD_ENABLED = True
    THINGSBOARD_HOST = "mqtt.thingsboard.cloud"
    THINGSBOARD_PORT = 1883
    THINGSBOARD_TOKEN = "YOUR_THINGSBOARD_ACCESS_TOKEN"
    THINGSBOARD_TOPIC = "v1/devices/me/telemetry"

    # --- Telegram Bot Configuration ---
    TELEGRAM_ENABLED = True
    TELEGRAM_BOT_TOKEN = "YOUR_TELEGRAM_BOT_TOKEN"
    TELEGRAM_CHAT_ID = "YOUR_TELEGRAM_CHAT_ID"
    ALERT_COOLDOWN_SEC = 15.0  # Minimum seconds between identical alerts

    # --- Loop Timing ---
    TELEMETRY_INTERVAL_SEC = 2.0
    LCD_ROTATE_INTERVAL_SEC = 3.0
    MAIN_LOOP_SLEEP_SEC = 0.02      # Fast protection loop period
    SENSOR_RETRY_SEC = 10.0         # Retry interval for dropped INA219


# ==============================================================================
# 2. GLOBAL LOCKS & TERMINAL + LCD LOGGER
# ==============================================================================
i2c_lock = threading.Lock()


class I2CLCD:
    """Self-contained PCF8574 I2C LCD Driver (No external LCD library required)."""
    LCD_CHR = 1
    LCD_CMD = 0
    LINE_ADDRS = [0x80, 0xC0, 0x94, 0xD4]
    LCD_BACKLIGHT = 0x08
    ENABLE = 0b00000100

    def __init__(self, bus_num=1, addr_primary=0x27, addr_secondary=0x3F, cols=16, rows=2):
        self.cols = cols
        self.rows = rows
        self.bus = None
        self.addr = None
        self.online = False

        if not SMBUS_AVAILABLE:
            return

        try:
            self.bus = smbus2.SMBus(bus_num)
            for candidate in (addr_primary, addr_secondary):
                try:
                    with i2c_lock:
                        self.bus.write_byte(candidate, self.LCD_BACKLIGHT)
                    self.addr = candidate
                    self.online = True
                    break
                except OSError:
                    continue

            if self.online:
                self._init_display()
        except Exception:
            self.online = False

    def _init_display(self):
        try:
            self._write_byte(0x33, self.LCD_CMD)
            self._write_byte(0x32, self.LCD_CMD)
            self._write_byte(0x06, self.LCD_CMD)
            self._write_byte(0x0C, self.LCD_CMD)
            self._write_byte(0x28, self.LCD_CMD)
            self._write_byte(0x01, self.LCD_CMD)
            time.sleep(0.005)
        except Exception:
            self.online = False

    def _toggle_enable(self, bits):
        time.sleep(0.0005)
        self.bus.write_byte(self.addr, (bits | self.ENABLE))
        time.sleep(0.0005)
        self.bus.write_byte(self.addr, (bits & ~self.ENABLE))
        time.sleep(0.0005)

    def _write_byte(self, bits, mode):
        if not self.online:
            return
        bits_high = mode | (bits & 0xF0) | self.LCD_BACKLIGHT
        bits_low = mode | ((bits << 4) & 0xF0) | self.LCD_BACKLIGHT
        with i2c_lock:
            self.bus.write_byte(self.addr, bits_high)
            self._toggle_enable(bits_high)
            self.bus.write_byte(self.addr, bits_low)
            self._toggle_enable(bits_low)

    def display(self, line1="", line2=""):
        if not self.online:
            return
        try:
            self._write_line(str(line1), 0)
            if self.rows > 1:
                self._write_line(str(line2), 1)
        except Exception:
            self.online = False

    def _write_line(self, text, row):
        text = text[:self.cols].ljust(self.cols)
        self._write_byte(self.LINE_ADDRS[row], self.LCD_CMD)
        for char in text:
            self._write_byte(ord(char), self.LCD_CHR)


class SystemLogger:
    """Unified logger that outputs to Terminal and updates the I2C LCD."""
    def __init__(self, lcd: I2CLCD):
        self.lcd = lcd
        self.override_until = 0.0
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s [%(levelname)s] %(message)s",
            datefmt="%H:%M:%S",
            handlers=[logging.StreamHandler(sys.stdout)]
        )
        self.logger = logging.getLogger("IoT_Guardian")

    def info(self, msg, lcd_l1=None, lcd_l2=None, hold_sec=0.0):
        self.logger.info(msg)
        if lcd_l1 is not None or lcd_l2 is not None:
            self.lcd_popup(lcd_l1 or "", lcd_l2 or "", hold_sec)

    def warning(self, msg, lcd_l1=None, lcd_l2=None, hold_sec=2.5):
        self.logger.warning(msg)
        if lcd_l1 is not None or lcd_l2 is not None:
            self.lcd_popup(lcd_l1 or "WARNING", lcd_l2 or msg[:16], hold_sec)

    def error(self, msg, lcd_l1=None, lcd_l2=None, hold_sec=4.0):
        self.logger.error(msg)
        if lcd_l1 is not None or lcd_l2 is not None:
            self.lcd_popup(lcd_l1 or "FAULT DETECTED!", lcd_l2 or msg[:16], hold_sec)

    def lcd_popup(self, line1, line2, hold_sec=2.5):
        if self.lcd and self.lcd.online:
            self.override_until = time.time() + hold_sec
            self.lcd.display(line1, line2)

    def lcd_routine_update(self, line1, line2):
        if self.lcd and self.lcd.online and time.time() >= self.override_until:
            self.lcd.display(line1, line2)


# ==============================================================================
# 3. CLOUD & ALERT SERVICES (TELEGRAM, HIVEMQ, THINGSBOARD)
# ==============================================================================
class CloudAndAlertManager:
    def __init__(self, logger: SystemLogger, command_callback=None):
        self.log = logger
        self.command_callback = command_callback
        self.alert_queue = queue.Queue()
        self.last_alert_times = {}

        self.hivemq_client = None
        self.hivemq_connected = False
        self.tb_client = None
        self.tb_connected = False

        self._init_mqtt_clients()
        self.worker_thread = threading.Thread(target=self._alert_worker, daemon=True)
        self.worker_thread.start()

    def _create_mqtt_client(self, client_id_prefix):
        cid = f"{client_id_prefix}_{int(time.time())}"
        try:
            return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=cid)
        except AttributeError:
            return mqtt.Client(client_id=cid)

    def _init_mqtt_clients(self):
        if not MQTT_AVAILABLE:
            self.log.warning("paho-mqtt not installed. Skipping HiveMQ & ThingsBoard.")
            return

        # 1. HiveMQ Cloud (TLS)
        if Config.HIVEMQ_ENABLED and "your-cluster-id" not in Config.HIVEMQ_HOST:
            try:
                self.hivemq_client = self._create_mqtt_client("rpi4_hivemq")
                self.hivemq_client.username_pw_set(Config.HIVEMQ_USER, Config.HIVEMQ_PASS)
                self.hivemq_client.tls_set(cert_reqs=ssl.CERT_REQUIRED)
                self.hivemq_client.on_connect = self._on_hivemq_connect
                self.hivemq_client.on_disconnect = self._on_hivemq_disconnect
                self.hivemq_client.on_message = self._on_hivemq_message
                self.hivemq_client.connect_async(Config.HIVEMQ_HOST, Config.HIVEMQ_PORT, keepalive=60)
                self.hivemq_client.loop_start()
                self.log.info(f"HiveMQ Cloud connecting to {Config.HIVEMQ_HOST}...")
            except Exception as e:
                self.log.warning(f"HiveMQ setup skipped/failed: {e}")
        else:
            self.log.info("HiveMQ placeholders detected; running offline until configured.")

        # 2. ThingsBoard
        if Config.THINGSBOARD_ENABLED and "YOUR_THINGSBOARD" not in Config.THINGSBOARD_TOKEN:
            try:
                self.tb_client = self._create_mqtt_client("rpi4_thingsboard")
                self.tb_client.username_pw_set(Config.THINGSBOARD_TOKEN)
                self.tb_client.on_connect = self._on_tb_connect
                self.tb_client.on_disconnect = self._on_tb_disconnect
                self.tb_client.connect_async(Config.THINGSBOARD_HOST, Config.THINGSBOARD_PORT, keepalive=60)
                self.tb_client.loop_start()
                self.log.info(f"ThingsBoard connecting to {Config.THINGSBOARD_HOST}...")
            except Exception as e:
                self.log.warning(f"ThingsBoard setup skipped/failed: {e}")
        else:
            self.log.info("ThingsBoard token placeholder detected; skipping direct connection.")

    @staticmethod
    def _rc_ok(rc):
        if isinstance(rc, int):
            return rc == 0
        return getattr(rc, "value", 0) == 0

    def _on_hivemq_connect(self, client, userdata, flags, rc, *args):
        if self._rc_ok(rc):
            self.hivemq_connected = True
            client.subscribe(Config.HIVEMQ_TOPIC_COMMAND)
            self.log.info("Connected to HiveMQ Cloud & subscribed to command topic.")
        else:
            self.log.warning(f"HiveMQ connection returned code: {rc}")

    def _on_hivemq_disconnect(self, client, userdata, *args):
        self.hivemq_connected = False
        self.log.warning("Disconnected from HiveMQ Cloud.")

    def _on_hivemq_message(self, client, userdata, msg):
        try:
            payload_raw = msg.payload.decode("utf-8").strip()
            self.log.info(f"MQTT Command Received: {payload_raw}")
            if self.command_callback:
                self.command_callback(payload_raw)
        except Exception as e:
            self.log.warning(f"Failed to parse MQTT command: {e}")

    def _on_tb_connect(self, client, userdata, flags, rc, *args):
        if self._rc_ok(rc):
            self.tb_connected = True
            self.log.info("Connected to ThingsBoard MQTT.")

    def _on_tb_disconnect(self, client, userdata, *args):
        self.tb_connected = False

    def publish_telemetry(self, data: dict):
        payload = json.dumps(data)
        if self.hivemq_client and self.hivemq_connected:
            try:
                self.hivemq_client.publish(Config.HIVEMQ_TOPIC_TELEMETRY, payload, qos=0)
            except Exception:
                pass

        if self.tb_client and self.tb_connected:
            try:
                # ThingsBoard ignores nested objects/None poorly; send flat numeric/bool/str values
                flat = {k: v for k, v in data.items() if v is not None and not isinstance(v, dict)}
                self.tb_client.publish(Config.THINGSBOARD_TOPIC, json.dumps(flat), qos=0)
            except Exception:
                pass

    def trigger_alert(self, alert_key: str, message: str, critical: bool = True):
        now = time.time()
        last_sent = self.last_alert_times.get(alert_key, 0.0)
        if now - last_sent < Config.ALERT_COOLDOWN_SEC:
            return
        self.last_alert_times[alert_key] = now

        alert_payload = {
            "timestamp": datetime.now().isoformat(),
            "type": alert_key,
            "severity": "CRITICAL" if critical else "WARNING",
            "message": message
        }

        if self.hivemq_client and self.hivemq_connected:
            try:
                self.hivemq_client.publish(Config.HIVEMQ_TOPIC_ALERTS, json.dumps(alert_payload), qos=1)
            except Exception:
                pass

        self.alert_queue.put(alert_payload)

    def _alert_worker(self):
        while True:
            alert = self.alert_queue.get()
            if alert is None:
                break
            if (
                Config.TELEGRAM_ENABLED
                and REQUESTS_AVAILABLE
                and "YOUR_TELEGRAM" not in Config.TELEGRAM_BOT_TOKEN
            ):
                try:
                    icon = "🚨" if alert["severity"] == "CRITICAL" else "⚠️"
                    # Plain text (no Markdown) so underscores in names can't break delivery
                    text = (
                        f"{icon} IoT Predictive Maintenance Alert\n"
                        f"Type: {alert['type']}\n"
                        f"Severity: {alert['severity']}\n"
                        f"Details: {alert['message']}\n"
                        f"Time: {alert['timestamp']}"
                    )
                    url = f"https://api.telegram.org/bot{Config.TELEGRAM_BOT_TOKEN}/sendMessage"
                    requests.post(
                        url,
                        json={"chat_id": Config.TELEGRAM_CHAT_ID, "text": text},
                        timeout=5
                    )
                except Exception as e:
                    self.log.warning(f"Telegram dispatch failed: {e}")
            self.alert_queue.task_done()

    def stop(self):
        for client in (self.hivemq_client, self.tb_client):
            if client:
                try:
                    client.loop_stop()
                except Exception:
                    pass


# ==============================================================================
# 4. EDGE IMPULSE & FFT PREDICTIVE MAINTENANCE ENGINE
# ==============================================================================
class EdgePredictiveEngine:
    """
    Runs FFT spectral feature extraction on MPU6050 vibration (preferred) or
    INA219 current waveforms, and evaluates via Edge Impulse Linux Runner (.eim)
    with automatic fallback to analytical FFT/RMS anomaly detection.
    """
    def __init__(self, logger: SystemLogger):
        self.log = logger
        self.runner = None
        self.ei_active = False
        self.ei_labels = []
        self._init_edge_impulse()

    def _init_edge_impulse(self):
        if not EDGE_IMPULSE_AVAILABLE:
            self.log.info("edge_impulse_linux not installed; using built-in FFT analyzer.")
            return

        if not os.path.exists(Config.EI_MODEL_PATH):
            self.log.info(f"Edge Impulse model '{Config.EI_MODEL_PATH}' not found; using built-in FFT analyzer.")
            return

        try:
            self.runner = ImpulseRunner(Config.EI_MODEL_PATH)
            model_info = self.runner.init()
            self.ei_labels = model_info.get("model_parameters", {}).get("labels", [])
            self.ei_active = True
            self.log.info(f"Edge Impulse Model Loaded! Labels: {self.ei_labels}")
        except Exception as e:
            self.log.warning(f"Edge Impulse init failed ({e}); falling back to built-in FFT analyzer.")
            self.ei_active = False

    def analyze_window(self, accel_mag_samples, current_samples):
        result = {
            "dominant_freq_hz": 0.0,
            "spectral_energy": 0.0,
            "vibration_rms": 0.0,
            "ei_prediction": "NO_DATA",
            "fault_probability": 0.0,
            "is_anomaly": False,
            "signal_source": "none"
        }

        # Choose primary signal (vibration preferred, current fallback)
        if len(accel_mag_samples) >= 8:
            signal, source = accel_mag_samples, "vibration"
            rms_limit = Config.VIBRATION_RMS_LIMIT
            energy_limit = Config.FFT_HARMONIC_ENERGY_LIMIT
        elif len(current_samples) >= 8:
            signal, source = current_samples, "current"
            rms_limit = Config.CURRENT_AC_RMS_LIMIT_A
            energy_limit = Config.CURRENT_FFT_ENERGY_LIMIT
        else:
            return result

        result["signal_source"] = source

        if NUMPY_AVAILABLE:
            arr = np.array(signal, dtype=np.float64)
            arr_ac = arr - np.mean(arr)  # Remove gravity / DC offset
            rms = float(np.sqrt(np.mean(np.square(arr_ac))))

            windowed = arr_ac * np.hanning(len(arr_ac))
            fft_vals = np.abs(np.fft.rfft(windowed)) / len(arr_ac)
            freqs = np.fft.rfftfreq(len(arr_ac), d=1.0 / Config.FFT_SAMPLE_RATE_HZ)

            if len(fft_vals) > 1:
                peak_idx = int(np.argmax(fft_vals[1:])) + 1
                dom_freq = float(freqs[peak_idx])
                spectral_energy = float(np.sum(np.square(fft_vals[1:])) * 100.0)
            else:
                dom_freq = 0.0
                spectral_energy = 0.0
        else:
            mean_val = sum(signal) / len(signal)
            ac_sig = [x - mean_val for x in signal]
            rms = math.sqrt(sum(x * x for x in ac_sig) / len(ac_sig))
            dom_freq = 0.0
            spectral_energy = rms * 10.0

        result["dominant_freq_hz"] = round(dom_freq, 2)
        result["spectral_energy"] = round(spectral_energy, 2)
        result["vibration_rms"] = round(rms, 3)

        # 1. Edge Impulse classifier (only if BOTH signals are present and full-length,
        #    so the feature layout matches training: [accel, current, accel, current, ...])
        if (
            self.ei_active and self.runner
            and len(accel_mag_samples) == Config.FFT_WINDOW_SIZE
            and len(current_samples) == Config.FFT_WINDOW_SIZE
        ):
            try:
                features = []
                for a, c in zip(accel_mag_samples, current_samples):
                    features.extend([float(a), float(c)])
                res = self.runner.classify(features)
                classifications = res.get("result", {}).get("classification", {})
                anomaly_score = float(res.get("result", {}).get("anomaly", 0.0))

                best_label, best_prob = "NORMAL", 0.0
                for label, prob in classifications.items():
                    if prob > best_prob:
                        best_prob, best_label = prob, label

                result["ei_prediction"] = best_label
                result["fault_probability"] = round(max(best_prob, anomaly_score), 3)
                if (
                    best_label.upper() not in ("NORMAL", "IDLE", "OK", "HEALTHY")
                    and best_prob >= Config.EI_ANOMALY_THRESHOLD
                ) or anomaly_score >= Config.EI_ANOMALY_THRESHOLD:
                    result["is_anomaly"] = True
                return result
            except Exception as e:
                self.log.warning(f"Edge Impulse inference error: {e}")

        # 2. Analytical FFT / RMS fallback (limits depend on signal source)
        fault_score = min(1.0, max(rms / rms_limit, spectral_energy / energy_limit))
        result["fault_probability"] = round(fault_score, 3)

        if rms >= rms_limit or spectral_energy >= energy_limit:
            result["ei_prediction"] = "FFT_BEARING_OR_LOAD_FAULT"
            result["is_anomaly"] = True
        elif fault_score >= 0.75:
            result["ei_prediction"] = "FFT_EARLY_WEAR_WARNING"
        else:
            result["ei_prediction"] = "HEALTHY"

        return result

    def stop(self):
        if self.runner:
            try:
                self.runner.stop()
            except Exception:
                pass


# ==============================================================================
# 5. FAULT-TOLERANT HARDWARE MANAGER
# ==============================================================================
class HardwareManager:
    """
    Initializes and polls all sensors/actuators independently.
    If any component is missing or disconnects mid-run, it logs a warning and
    continues running with the remaining hardware.
    """
    def __init__(self, logger: SystemLogger):
        self.log = logger
        self.state_lock = threading.RLock()   # MQTT / console / main thread all touch motor state
        self.boot_time = time.time()

        self.motor_on = False
        self.motor_on_since = 0.0
        self.fault_latched = False
        self.fault_reason = "NONE"
        self.was_on_before_trip = False

        # Hardware handles & online flags
        self.gpio_ok = False
        self.i2c_bus = None
        self.raw_smbus = None

        self.ina219 = None
        self.ina219_online = False
        self.ina219_was_online = False
        self.last_ina_retry = 0.0

        self.mpu6050 = None
        self.mpu6050_online = False
        self.mpu6050_raw_mode = False

        self.dht22 = None
        self.dht22_online = False
        self.last_dht_read = 0.0
        self.cached_temp = None
        self.cached_hum = None

        self.pn532 = None
        self.pn532_online = False
        self.last_nfc_tap_time = 0.0
        self.nfc_tag_present = False
        self.nfc_last_seen = 0.0

        # MQ5 state (updated only by update_gas(), called from the fast loop)
        self.gas_streak = 0
        self.gas_confirmed = None     # None = no GPIO, True/False otherwise
        self.gas_warming_up = False

        self._init_all_hardware()

    # --------------------------------------------------------------------------
    # Initialization
    # --------------------------------------------------------------------------
    def _init_all_hardware(self):
        # 1. GPIO (MQ5 input + relay, relay kept OFF/floating from the very start)
        if GPIO_AVAILABLE:
            try:
                GPIO.setwarnings(False)
                GPIO.setmode(GPIO.BCM)
                GPIO.setup(Config.PIN_MQ5_DO, GPIO.IN, pull_up_down=GPIO.PUD_UP)
                self.gpio_ok = True
                self._drive_relay(False)  # No GPIO.OUT here -> no boot-time relay click
                self.gas_confirmed = False
                self.log.info("GPIO initialized (Relay & MQ5 ready).")
            except Exception as e:
                self.log.warning(f"GPIO init failed (skipping Relay/MQ5): {e}")
        else:
            self.log.warning("RPi.GPIO not available; skipping Relay & MQ5.")

        # 2. Blinka I2C & Raw SMBus
        if BLINKA_AVAILABLE:
            try:
                self.i2c_bus = busio.I2C(board.SCL, board.SDA)
            except Exception as e:
                self.log.warning(f"Blinka I2C init failed: {e}")

        if SMBUS_AVAILABLE:
            try:
                self.raw_smbus = smbus2.SMBus(Config.I2C_BUS_NUM)
            except Exception as e:
                self.log.warning(f"SMBus init failed: {e}")

        # 3. INA219 Current/Voltage Sensor
        self._init_ina219()

        # 4. MPU6050 (Adafruit lib with Raw SMBus fallback)
        if MPU6050_LIB_AVAILABLE and self.i2c_bus:
            try:
                with i2c_lock:
                    self.mpu6050 = adafruit_mpu6050.MPU6050(self.i2c_bus, address=Config.ADDR_MPU6050)
                self.mpu6050_online = True
                self.log.info("MPU6050 Vibration Sensor (Adafruit): ONLINE")
            except Exception as e:
                self.log.info(f"MPU6050 Adafruit init failed ({e}); trying raw SMBus...")

        if not self.mpu6050_online and self.raw_smbus:
            try:
                with i2c_lock:
                    self.raw_smbus.write_byte_data(Config.ADDR_MPU6050, 0x6B, 0x00)  # Wake up
                self.mpu6050_online = True
                self.mpu6050_raw_mode = True
                self.log.info("MPU6050 Vibration Sensor (Raw SMBus): ONLINE")
            except Exception:
                self.log.warning(f"MPU6050 not detected at 0x{Config.ADDR_MPU6050:02X} (skipping).")

        # 5. PN532 NFC Module over I2C
        if PN532_LIB_AVAILABLE and self.i2c_bus:
            try:
                with i2c_lock:
                    self.pn532 = PN532_I2C(self.i2c_bus, address=Config.ADDR_PN532, debug=False)
                    ic, ver, rev, support = self.pn532.firmware_version
                    self.pn532.SAM_configuration()
                self.pn532_online = True
                self.log.info(f"PN532 NFC Reader (v{ver}.{rev}): ONLINE")
            except Exception:
                self.log.warning(f"PN532 NFC not detected at 0x{Config.ADDR_PN532:02X} (skipping).")
        else:
            self.log.warning("PN532 library or I2C unavailable (skipping PN532).")

        # 6. DHT22 Temperature & Humidity Sensor
        if DHT_LIB_AVAILABLE and BLINKA_AVAILABLE:
            try:
                dht_pin = getattr(board, f"D{Config.PIN_DHT22}")
                self.dht22 = adafruit_dht.DHT22(dht_pin, use_pulseio=False)
                self.dht22_online = True
                self.log.info(f"DHT22 Sensor on GPIO{Config.PIN_DHT22}: ONLINE")
            except Exception as e:
                self.log.warning(f"DHT22 init failed on GPIO{Config.PIN_DHT22} (skipping): {e}")
        else:
            self.log.warning("adafruit_dht not available (skipping DHT22).")

    def _init_ina219(self):
        if not (INA219_LIB_AVAILABLE and self.i2c_bus):
            if self.last_ina_retry == 0.0:
                self.log.warning("INA219 library or I2C unavailable (skipping INA219).")
            return False
        try:
            with i2c_lock:
                self.ina219 = adafruit_ina219.INA219(self.i2c_bus, addr=Config.ADDR_INA219)
            self.ina219_online = True
            self.ina219_was_online = True
            self.log.info("INA219 Current/Voltage Sensor: ONLINE")
            return True
        except Exception as e:
            if self.last_ina_retry == 0.0:
                self.log.warning(f"INA219 not detected at 0x{Config.ADDR_INA219:02X} (will keep retrying): {e}")
            return False

    def maintenance(self):
        """Periodic housekeeping: retry sensors that dropped off the bus."""
        now = time.time()
        if not self.ina219_online and now - self.last_ina_retry >= Config.SENSOR_RETRY_SEC:
            self.last_ina_retry = now
            self._init_ina219()

    # --------------------------------------------------------------------------
    # Relay & Motor Control
    # --------------------------------------------------------------------------
    def _drive_relay(self, on: bool):
        """
        Open-drain style drive for 5V relay modules on 3.3V Pi GPIO.
        Active-low: LOW = ON, pin floats (input) = OFF. The pin is never driven HIGH,
        so a 3.3V HIGH can't leave the relay half-energised.
        """
        if Config.RELAY_ACTIVE_LOW:
            if on:
                GPIO.setup(Config.PIN_RELAY, GPIO.OUT)
                GPIO.output(Config.PIN_RELAY, GPIO.LOW)
            else:
                GPIO.setup(Config.PIN_RELAY, GPIO.IN)
        else:
            GPIO.setup(Config.PIN_RELAY, GPIO.OUT)
            GPIO.output(Config.PIN_RELAY, GPIO.HIGH if on else GPIO.LOW)

    def clear_fault(self):
        with self.state_lock:
            self.fault_latched = False
            self.fault_reason = "NONE"
            self.was_on_before_trip = False

    def set_motor_relay(self, turn_on: bool, reason: str = "Manual/NFC"):
        with self.state_lock:
            if turn_on and self.fault_latched:
                self.log.warning(
                    f"Cannot start motor: fault '{self.fault_reason}' is latched. Reset first.",
                    lcd_l1="FAULT LATCHED",
                    lcd_l2="RESET FIRST",
                    hold_sec=2.0
                )
                return False

            self.motor_on = turn_on
            if turn_on:
                self.motor_on_since = time.time()

            if self.gpio_ok:
                try:
                    self._drive_relay(turn_on)
                except Exception as e:
                    self.log.warning(f"Relay GPIO write error: {e}")

            state_str = "ON" if turn_on else "OFF"
            self.log.info(
                f"Motor Relay -> {state_str} (Trigger: {reason})",
                lcd_l1=f"MOTOR: {state_str}",
                lcd_l2=f"BY: {reason[:12]}",
                hold_sec=2.0
            )
            return self.motor_on

    def emergency_trip(self, fault_name: str, detail: str):
        with self.state_lock:
            if not self.fault_latched:
                self.was_on_before_trip = self.motor_on
            self.fault_latched = True
            self.fault_reason = fault_name
            self.motor_on = False
            if self.gpio_ok:
                try:
                    self._drive_relay(False)
                except Exception:
                    pass
        self.log.error(
            f"EMERGENCY TRIP [{fault_name}]: {detail}",
            lcd_l1=f"!{fault_name[:14]}!",
            lcd_l2=detail[:16],
            hold_sec=5.0
        )

    # --------------------------------------------------------------------------
    # Sensor Polling Methods
    # --------------------------------------------------------------------------
    def poll_nfc(self):
        """Checks PN532 for a card tap and toggles the motor state."""
        if not self.pn532_online or not self.pn532:
            return None

        try:
            with i2c_lock:
                uid = self.pn532.read_passive_target(timeout=0.04)
            now = time.time()
            if uid is None:
                # A tag counts as removed only after a short absence (reads can flicker)
                if self.nfc_tag_present and now - self.nfc_last_seen > Config.NFC_REMOVE_GRACE_SEC:
                    self.nfc_tag_present = False
                return None

            self.nfc_last_seen = now
            if self.nfc_tag_present:
                return None  # Same tag still on the reader - ignore until it is removed
            self.nfc_tag_present = True

            if now - self.last_nfc_tap_time < Config.NFC_DEBOUNCE_SECONDS:
                return None
            self.last_nfc_tap_time = now

            uid_hex = "".join([f"{b:02x}" for b in uid])
            authorized_list = [u.lower() for u in Config.AUTHORIZED_NFC_UIDS]

            if not authorized_list or uid_hex.lower() in authorized_list:
                if not authorized_list:
                    self.log.info(f"NFC tag UID (open mode): {uid_hex}")
                with self.state_lock:
                    if self.fault_latched:
                        self.clear_fault()  # Tap clears fault, then toggles (-> starts)
                    new_state = not self.motor_on
                    self.set_motor_relay(new_state, reason=f"NFC:{uid_hex[:8]}")
                return {"uid": uid_hex, "authorized": True, "motor_state": "ON" if self.motor_on else "OFF"}

            self.log.warning(
                f"Unauthorized NFC Tag scanned: {uid_hex}",
                lcd_l1="NFC DENIED!",
                lcd_l2=f"UID:{uid_hex[:12]}",
                hold_sec=2.5
            )
            return {"uid": uid_hex, "authorized": False, "motor_state": "ON" if self.motor_on else "OFF"}
        except Exception as e:
            self.log.warning(f"PN532 read glitch (ignoring): {e}")
            return None

    def _mark_ina_lost(self, e):
        self.log.warning(f"INA219 disconnected or read error: {e}")
        self.ina219_online = False
        self.last_ina_retry = time.time()

    def read_current_fast(self):
        """Current-only read (amps) for the fast protection loop and FFT burst."""
        if not self.ina219_online or not self.ina219:
            return None
        try:
            with i2c_lock:
                current_ma = float(self.ina219.current)
            return round(abs(current_ma) / 1000.0, 3)
        except Exception as e:
            self._mark_ina_lost(e)
            return None

    def read_ina219(self):
        empty = {"voltage_v": None, "current_a": None, "power_w": None}
        if not self.ina219_online or not self.ina219:
            return empty
        try:
            with i2c_lock:
                bus_v = float(self.ina219.bus_voltage)
                shunt_v = float(self.ina219.shunt_voltage)
                current_ma = float(self.ina219.current)
                power_mw = float(self.ina219.power)

            return {
                "voltage_v": round(bus_v + shunt_v, 3),   # adafruit shunt_voltage is already in volts
                "current_a": round(abs(current_ma) / 1000.0, 3),
                "power_w": round(abs(power_mw) / 1000.0, 3),
            }
        except Exception as e:
            self._mark_ina_lost(e)
            return empty

    def read_mpu6050(self):
        empty = {"ax": None, "ay": None, "az": None, "gx": None, "gy": None, "gz": None, "accel_mag": None}
        if not self.mpu6050_online:
            return empty
        try:
            if not self.mpu6050_raw_mode and self.mpu6050:
                with i2c_lock:
                    ax, ay, az = self.mpu6050.acceleration
                    gx, gy, gz = self.mpu6050.gyro
            else:
                def _read_word(reg):
                    high = self.raw_smbus.read_byte_data(Config.ADDR_MPU6050, reg)
                    low = self.raw_smbus.read_byte_data(Config.ADDR_MPU6050, reg + 1)
                    val = (high << 8) + low
                    return -((65535 - val) + 1) if val >= 0x8000 else val

                with i2c_lock:
                    ax = (_read_word(0x3B) / 16384.0) * 9.80665
                    ay = (_read_word(0x3D) / 16384.0) * 9.80665
                    az = (_read_word(0x3F) / 16384.0) * 9.80665
                    gx = _read_word(0x43) / 131.0
                    gy = _read_word(0x45) / 131.0
                    gz = _read_word(0x47) / 131.0

            accel_mag = math.sqrt(ax * ax + ay * ay + az * az)
            return {
                "ax": round(ax, 3), "ay": round(ay, 3), "az": round(az, 3),
                "gx": round(gx, 3), "gy": round(gy, 3), "gz": round(gz, 3),
                "accel_mag": round(accel_mag, 3)
            }
        except Exception as e:
            self.log.warning(f"MPU6050 disconnected or read error: {e}")
            self.mpu6050_online = False
            return empty

    def read_dht22(self):
        if not self.dht22_online or not self.dht22:
            return {"temperature_c": None, "humidity_pct": None}

        now = time.time()
        if now - self.last_dht_read < 2.2:  # DHT22 needs >= 2 s between reads
            return {"temperature_c": self.cached_temp, "humidity_pct": self.cached_hum}

        self.last_dht_read = now
        try:
            temp = self.dht22.temperature
            hum = self.dht22.humidity
            if temp is not None:
                self.cached_temp = round(float(temp), 1)
            if hum is not None:
                self.cached_hum = round(float(hum), 1)
        except RuntimeError:
            pass  # Normal DHT checksum/timing miss; keep cached values
        except Exception as e:
            self.log.warning(f"DHT22 hardware error (disabling): {e}")
            self.dht22_online = False

        return {"temperature_c": self.cached_temp, "humidity_pct": self.cached_hum}

    def update_gas(self):
        """
        Fast, GPIO-only MQ5 check. Call every loop cycle. Handles heater warm-up and
        debouncing. Result lives in self.gas_confirmed / self.gas_warming_up.
        """
        if not self.gpio_ok:
            self.gas_confirmed = None
            return

        self.gas_warming_up = (time.time() - self.boot_time) < Config.MQ5_WARMUP_SEC
        try:
            raw_val = GPIO.input(Config.PIN_MQ5_DO)
        except Exception:
            self.gas_confirmed = None
            return

        raw_detected = (raw_val == GPIO.LOW) if Config.MQ5_ACTIVE_LOW else (raw_val == GPIO.HIGH)

        if self.gas_warming_up:
            self.gas_streak = 0
            self.gas_confirmed = False
            return

        self.gas_streak = self.gas_streak + 1 if raw_detected else 0
        self.gas_confirmed = self.gas_streak >= Config.GAS_DEBOUNCE_READS

    def capture_fft_burst(self, overcurrent_cb=None):
        """
        Captures a window of vibration & current samples for FFT / Edge Impulse.
        overcurrent_cb(current_a) -> True if it tripped (burst aborts immediately).
        """
        accel_samples, current_samples = [], []

        if not self.mpu6050_online and not self.ina219_online:
            return accel_samples, current_samples

        sample_delay = 1.0 / float(Config.FFT_SAMPLE_RATE_HZ)
        for _ in range(Config.FFT_WINDOW_SIZE):
            t_start = time.time()
            if self.mpu6050_online:
                mpu = self.read_mpu6050()
                if mpu["accel_mag"] is not None:
                    accel_samples.append(mpu["accel_mag"])
            if self.ina219_online:
                cur = self.read_current_fast()
                if cur is not None:
                    current_samples.append(cur)
                    if overcurrent_cb and overcurrent_cb(cur):
                        break
            elapsed = time.time() - t_start
            if elapsed < sample_delay:
                time.sleep(sample_delay - elapsed)

        return accel_samples, current_samples

    def cleanup(self):
        try:
            self.set_motor_relay(False, reason="System Shutdown")
        except Exception:
            pass
        if self.dht22:
            try:
                self.dht22.exit()
            except Exception:
                pass
        if self.gpio_ok:
            try:
                self._drive_relay(False)
                GPIO.cleanup()
            except Exception:
                pass


# ==============================================================================
# 6. MAIN APPLICATION CONTROLLER
# ==============================================================================
class GraduationProjectController:
    def __init__(self):
        self.lcd = I2CLCD(
            bus_num=Config.I2C_BUS_NUM,
            addr_primary=Config.ADDR_LCD_PRIMARY,
            addr_secondary=Config.ADDR_LCD_SECONDARY,
            cols=Config.LCD_COLS,
            rows=Config.LCD_ROWS
        )
        self.log = SystemLogger(self.lcd)
        self.log.info(
            "Booting Overcurrent & Predictive Maintenance System...",
            lcd_l1="IOT GRAD PROJECT",
            lcd_l2="BOOTING SYSTEM..",
            hold_sec=2.0
        )

        self.hw = HardwareManager(self.log)
        self.edge_ai = EdgePredictiveEngine(self.log)
        self.cloud = CloudAndAlertManager(self.log, command_callback=self.handle_remote_command)

        self.running = True
        self.lcd_page = 0
        self.last_lcd_switch = 0.0
        self.last_telemetry_time = 0.0
        self.last_telemetry = {}
        self._oc_streak = 0

    # --------------------------------------------------------------------------
    # Commands (MQTT / Node-RED / terminal)
    # --------------------------------------------------------------------------
    def handle_remote_command(self, cmd_str: str, source: str = "Node-RED/MQTT"):
        try:
            data = json.loads(cmd_str) if cmd_str.startswith("{") else {"motor": cmd_str.upper()}
        except Exception:
            data = {"motor": cmd_str.upper()}

        action = str(data.get("motor", "")).upper()
        if action in ("ON", "START", "1", "TRUE"):
            self.hw.clear_fault()
            self.hw.set_motor_relay(True, reason=source)
        elif action in ("OFF", "STOP", "0", "FALSE"):
            self.hw.set_motor_relay(False, reason=source)
        elif action == "RESET":
            self.hw.clear_fault()
            self.log.info("Fault state reset.", lcd_l1="REMOTE RESET", lcd_l2="SYSTEM READY", hold_sec=2.0)

    def _console_loop(self):
        """Bench control from the terminal: on | off | reset | status | quit"""
        if not sys.stdin or not sys.stdin.isatty():
            return
        self.log.info("Console commands: on | off | reset | status | quit")
        while self.running:
            try:
                line = sys.stdin.readline()
            except Exception:
                break
            if not line:
                break
            cmd = line.strip().lower()
            if cmd in ("on", "off", "reset"):
                self.handle_remote_command(cmd, source="Console")
            elif cmd == "status":
                print(json.dumps(self.last_telemetry, indent=2, default=str))
            elif cmd in ("quit", "exit", "q"):
                self.running = False

    # --------------------------------------------------------------------------
    # Protection logic
    # --------------------------------------------------------------------------
    def check_overcurrent(self, current_a):
        """Returns True if it tripped. Called from the fast loop AND inside the FFT burst."""
        hw = self.hw
        if current_a is None or not hw.motor_on:
            self._oc_streak = 0
            return False
        if time.time() - hw.motor_on_since < Config.START_INRUSH_GRACE_SEC:
            self._oc_streak = 0
            return False

        if current_a >= Config.OVERCURRENT_LIMIT_A:
            self._oc_streak += 1
            if self._oc_streak >= Config.OVERCURRENT_CONFIRM_SAMPLES:
                self._oc_streak = 0
                hw.emergency_trip("OVERCURRENT", f"I={current_a:.2f}A>{Config.OVERCURRENT_LIMIT_A}A")
                self.cloud.trigger_alert(
                    "OVERCURRENT_TRIP",
                    f"Motor tripped! Current reached {current_a:.2f}A (Limit: {Config.OVERCURRENT_LIMIT_A}A).",
                    critical=True
                )
                return True
        else:
            self._oc_streak = 0
        return False

    def check_gas(self):
        hw = self.hw
        hw.update_gas()

        if hw.gas_confirmed is True:
            if not hw.fault_latched:
                hw.emergency_trip("GAS_LEAK", "MQ5 Triggered!")
            self.cloud.trigger_alert(
                "GAS_SMOKE_DETECTED",
                "MQ5 detected gas or smoke near the motor unit! Relay tripped.",
                critical=True
            )
        elif (
            hw.gas_confirmed is False
            and Config.AUTO_CLEAR_GAS_FAULT
            and hw.fault_latched
            and hw.fault_reason == "GAS_LEAK"
        ):
            resume = hw.was_on_before_trip
            hw.clear_fault()
            self.log.info("Air clear - gas fault auto-cleared.", lcd_l1="AIR CLEAR", lcd_l2="FAULT CLEARED", hold_sec=2.0)
            if resume:
                hw.set_motor_relay(True, reason="AutoRecover")

    def fast_protection_check(self):
        """Runs every loop cycle (~20-50 ms): gas (GPIO) + current (single I2C read)."""
        self.check_gas()
        if self.hw.motor_on and self.hw.ina219_online:
            self.check_overcurrent(self.hw.read_current_fast())

    # --------------------------------------------------------------------------
    # LCD
    # --------------------------------------------------------------------------
    def update_lcd_carousel(self, telemetry: dict):
        now = time.time()
        if now - self.last_lcd_switch >= Config.LCD_ROTATE_INTERVAL_SEC:
            self.lcd_page = (self.lcd_page + 1) % 4
            self.last_lcd_switch = now

        if self.hw.fault_latched:
            self.log.lcd_routine_update(f"FAULT:{self.hw.fault_reason[:10]}", "SEND RESET/TAP")
            return

        def fmt(v, spec, suffix=""):
            return f"{v:{spec}}{suffix}" if v is not None else "N/A"

        m_state = "ON " if self.hw.motor_on else "OFF"
        curr = fmt(telemetry["current_a"], ".2f", "A")
        volt = fmt(telemetry["voltage_v"], ".1f", "V")
        power = fmt(telemetry["power_w"], ".1f", "W")
        temp = fmt(telemetry["temperature_c"], ".1f", "C")
        hum = fmt(telemetry["humidity_pct"], ".0f", "%")
        vib = fmt(telemetry["vibration_rms"], ".2f")
        freq = fmt(telemetry["dominant_freq_hz"], ".0f", "Hz")

        if self.lcd_page == 0:
            self.log.lcd_routine_update(f"M:{m_state} I:{curr}", f"V:{volt} P:{power}")
        elif self.lcd_page == 1:
            self.log.lcd_routine_update(f"TEMP: {temp}", f"HUM:  {hum}")
        elif self.lcd_page == 2:
            self.log.lcd_routine_update(f"VIB RMS: {vib}", f"FFT PK:  {freq}")
        else:
            if telemetry["gas_warming_up"]:
                gas_txt = "WARMUP"
            elif telemetry["gas_detected"]:
                gas_txt = "LEAK!"
            else:
                gas_txt = "SAFE"
            self.log.lcd_routine_update(f"GAS: {gas_txt}", f"AI: {str(telemetry['ei_prediction'])[:12]}")

    # --------------------------------------------------------------------------
    # Main loop
    # --------------------------------------------------------------------------
    def run(self):
        self.log.info(
            "System Online. Tap NFC Tag, send an MQTT command, or type a console command.",
            lcd_l1="SYSTEM READY",
            lcd_l2="TAP NFC TO START",
            hold_sec=2.0
        )

        if Config.AUTO_START_MOTOR:
            self.hw.set_motor_relay(True, reason="AutoStart")

        threading.Thread(target=self._console_loop, daemon=True).start()

        try:
            while self.running:
                # 1. NFC tap check (snappy)
                nfc_event = self.hw.poll_nfc()
                if nfc_event and not nfc_event["authorized"]:
                    self.cloud.trigger_alert(
                        "UNAUTHORIZED_NFC",
                        f"Unauthorized NFC Tag scanned (UID: {nfc_event['uid']})",
                        critical=False
                    )

                # 2. Fast protection (gas + overcurrent) every cycle
                self.fast_protection_check()

                # 3. Periodic telemetry, slower protections & Edge AI FFT
                now = time.time()
                if now - self.last_telemetry_time >= Config.TELEMETRY_INTERVAL_SEC:
                    self.last_telemetry_time = now
                    self.hw.maintenance()

                    accel_window, current_window = self.hw.capture_fft_burst(overcurrent_cb=self.check_overcurrent)

                    ina_data = self.hw.read_ina219()
                    mpu_data = self.hw.read_mpu6050()
                    dht_data = self.hw.read_dht22()
                    fft_ai_data = self.edge_ai.analyze_window(accel_window, current_window)

                    current_a = ina_data["current_a"]
                    voltage_v = ina_data["voltage_v"]
                    temp_c = dht_data["temperature_c"]
                    gas_leak = self.hw.gas_confirmed

                    # A. High-current warning (trip itself is handled in the fast path)
                    if (
                        current_a is not None and self.hw.motor_on
                        and Config.WARNING_CURRENT_A <= current_a < Config.OVERCURRENT_LIMIT_A
                    ):
                        self.log.warning(
                            f"High Current Warning: {current_a:.2f}A",
                            lcd_l1="HIGH CURRENT WRN",
                            lcd_l2=f"I={current_a:.2f}A",
                            hold_sec=2.0
                        )
                        self.cloud.trigger_alert("HIGH_CURRENT_WARNING", f"Elevated current detected: {current_a:.2f}A.", critical=False)

                    # B. Undervoltage (motor ON only)
                    if self.hw.motor_on and voltage_v is not None and 0.5 < voltage_v < Config.UNDERVOLTAGE_LIMIT_V:
                        self.log.warning(f"Supply Undervoltage: {voltage_v:.2f}V")
                        self.cloud.trigger_alert("UNDERVOLTAGE_WARNING", f"Supply voltage dropped to {voltage_v:.2f}V.", critical=False)

                    # C. Thermal protection (DHT22 = ambient/enclosure proxy)
                    if temp_c is not None and temp_c >= Config.MAX_TEMP_C and not self.hw.fault_latched:
                        self.hw.emergency_trip("OVERHEAT", f"Temp={temp_c:.1f}C")
                        self.cloud.trigger_alert("OVERHEAT_TRIP", f"Thermal protection tripped! Temperature: {temp_c:.1f}C.", critical=True)

                    # D. Optional fail-safe: current sensing lost while motor runs
                    if (
                        Config.FAILSAFE_ON_INA_LOSS and self.hw.motor_on
                        and self.hw.ina219_was_online and not self.hw.ina219_online
                        and not self.hw.fault_latched
                    ):
                        self.hw.emergency_trip("SENSOR_LOST", "INA219 offline")
                        self.cloud.trigger_alert("INA219_LOST", "Current sensor lost while motor running. Relay tripped.", critical=True)

                    # E. Predictive maintenance (FFT + Edge Impulse)
                    if self.hw.motor_on and fft_ai_data["is_anomaly"]:
                        msg = (
                            f"Predictive Maintenance Anomaly: {fft_ai_data['ei_prediction']} "
                            f"[{fft_ai_data['signal_source']}] | "
                            f"RMS={fft_ai_data['vibration_rms']}, "
                            f"Dom Freq={fft_ai_data['dominant_freq_hz']} Hz, "
                            f"Energy={fft_ai_data['spectral_energy']}"
                        )
                        self.log.warning(msg, lcd_l1="PRED MAINT ALERT", lcd_l2=f"{fft_ai_data['ei_prediction'][:16]}", hold_sec=3.0)
                        self.cloud.trigger_alert("PREDICTIVE_MAINTENANCE_ANOMALY", msg, critical=False)

                    # Unified telemetry payload
                    telemetry = {
                        "timestamp": datetime.now().isoformat(),
                        "motor_on": self.hw.motor_on,
                        "fault_latched": self.hw.fault_latched,
                        "fault_reason": self.hw.fault_reason,
                        "voltage_v": voltage_v,
                        "current_a": current_a,
                        "power_w": ina_data["power_w"],
                        "temperature_c": temp_c,
                        "humidity_pct": dht_data["humidity_pct"],
                        "gas_detected": gas_leak,
                        "gas_warming_up": self.hw.gas_warming_up,
                        "ax": mpu_data["ax"],
                        "ay": mpu_data["ay"],
                        "az": mpu_data["az"],
                        "accel_mag": mpu_data["accel_mag"],
                        "vibration_rms": fft_ai_data["vibration_rms"],
                        "dominant_freq_hz": fft_ai_data["dominant_freq_hz"],
                        "spectral_energy": fft_ai_data["spectral_energy"],
                        "signal_source": fft_ai_data["signal_source"],
                        "ei_prediction": fft_ai_data["ei_prediction"],
                        "fault_probability": fft_ai_data["fault_probability"],
                        "sensors_online": {
                            "gpio": self.hw.gpio_ok,
                            "ina219": self.hw.ina219_online,
                            "mpu6050": self.hw.mpu6050_online,
                            "dht22": self.hw.dht22_online,
                            "pn532": self.hw.pn532_online,
                            "lcd": self.lcd.online
                        }
                    }
                    self.last_telemetry = telemetry

                    def na(v, suffix=""):
                        return f"{v}{suffix}" if v is not None else "N/A"

                    if gas_leak is None:
                        gas_txt = "N/A"
                    elif self.hw.gas_warming_up:
                        gas_txt = "WARMUP"
                    else:
                        gas_txt = "LEAK" if gas_leak else "OK"

                    self.log.info(
                        f"STATE={'ON' if self.hw.motor_on else 'OFF'}"
                        f"{'/FAULT:' + self.hw.fault_reason if self.hw.fault_latched else ''} | "
                        f"V={na(voltage_v, 'V')} | I={na(current_a, 'A')} | "
                        f"T={na(temp_c, 'C')} | H={na(dht_data['humidity_pct'], '%')} | "
                        f"GAS={gas_txt} | VIB_RMS={fft_ai_data['vibration_rms']} | "
                    )

                    self.update_lcd_carousel(telemetry)
                    self.cloud.publish_telemetry(telemetry)

                time.sleep(Config.MAIN_LOOP_SLEEP_SEC)

        except KeyboardInterrupt:
            self.log.info("Shutting down on user interrupt (Ctrl+C)...", lcd_l1="SYSTEM SHUTDOWN", lcd_l2="GOODBYE!", hold_sec=2.0)
        finally:
            self.running = False
            self.edge_ai.stop()
            self.cloud.stop()
            self.hw.cleanup()


if __name__ == "__main__":
    app = GraduationProjectController()
    app.run()
