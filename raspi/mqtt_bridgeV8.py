#!/usr/bin/env python3
"""
Puente MQTT <-> brazo SO101 (líder / seguidor) - versión 8.0

  dashboard --so101/cmd---> [puente] --SYNC_WRITE--> LÍDER (LEADER_PORT)
                                                       |  (la copia al seguidor la hace so101d)
  dashboard <-so101/state-- [puente] <--READ---------- SEGUIDOR (FOLLOWER_PORT)
  dashboard <-so101/event-- [puente] <- cambios de modo y fallas del demonio
  modo y fallas REALES del demonio: [puente] --so101ctl status--> so101d (memoria compartida)  [MODE_SOURCE = ctl]
  cambio de modo desde el dashboard: SET_MODE --> [puente] --so101ctl <modo>--> so101d        [DAEMON_CONTROL = 1]
  modo de CONTROL (SET_CONTROL): 'local' = el usuario mueve el líder a mano y el puente NO acepta MOVE_JOINTS;
                                 'dashboard' = el dashboard comanda el líder. Por defecto: local.

Configuración:
- Parámetros generales: so101_bridge.ini
- Calibraciones líder/seguidor: so101_calibration.ini

El puente NUNCA escribe consignas en el seguidor: solo lo lee.
"""

import argparse
import configparser
import json
import logging
import math
import os
import re
import shutil
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

import paho.mqtt.client as mqtt
from paho.mqtt.enums import CallbackAPIVersion
import serial

logging.basicConfig(
    level=logging.INFO,
    format='[%(asctime)s] [%(levelname)s] %(message)s',
    datefmt='%H:%M:%S'
)
logger = logging.getLogger("SO101_Bridge")

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_INI = os.path.join(SCRIPT_DIR, "so101_bridge.ini")
DEFAULT_CALIB_INI = os.path.join(SCRIPT_DIR, "so101_calibration.ini")

TOPIC_CMD = "so101/cmd"
TOPIC_STATE = "so101/state"
TOPIC_EVENT = "so101/event"
TOPIC_RESULT = "so101/cmd/result"

# Modos de operación del demonio
MODES = ("teleop", "estop", "idle", "reset", "hold")
MODE_SOURCES = ("ctl", "cmd", "none")

# Modo de CONTROL del líder (no es el modo del demonio)
CONTROL_MODES = ("local", "dashboard")

EVENT_SKIP_FAULTS = {"EMERGENCY_STOP_TRIGGERED"}

LEADER_TORQUE = os.getenv("LEADER_TORQUE", "0") == "1"

JOINTS = {
    "shoulder_pan": 1,
    "shoulder_lift": 2,
    "elbow_flex": 3,
    "wrist_flex": 4,
    "wrist_roll": 5,
    "gripper": 6
}

# Diccionarios de calibración por defecto en caso de no hallar el .ini
CALIBRATION_LEADER = {
    "shoulder_pan":  {"LnormMin": -100, "LrawMin": 729,  "LnormMax": 100, "LrawMax": 4000},
    "shoulder_lift": {"LnormMin": -100, "LrawMin": 1822, "LnormMax": 100, "LrawMax": 3323},
    "elbow_flex":    {"LnormMin": -100, "LrawMin": 824,  "LnormMax": 100, "LrawMax": 3051},
    "wrist_flex":    {"LnormMin": -100, "LrawMin": 837,  "LnormMax": 100, "LrawMax": 3213},
    "wrist_roll":    {"LnormMin": -100, "LrawMin": 0,    "LnormMax": 100, "LrawMax": 4095},
    "gripper":       {"LnormMin": 0,    "LrawMin": 2040, "LnormMax": 100, "LrawMax": 3400}
}

CALIBRATION_FOLLOWER = {
    "shoulder_pan":  {"LnormMin": -100, "LrawMin": 700,  "LnormMax": 100, "LrawMax": 3900},
    "shoulder_lift": {"LnormMin": -100, "LrawMin": 760,  "LnormMax": 100, "LrawMax": 2350},
    "elbow_flex":    {"LnormMin": -100, "LrawMin": 923,  "LnormMax": 100, "LrawMax": 3051},
    "wrist_flex":    {"LnormMin": -100, "LrawMin": 850,  "LnormMax": 100, "LrawMax": 2700},
    "wrist_roll":    {"LnormMin": -100, "LrawMin": 0,    "LnormMax": 100, "LrawMax": 4095},
    "gripper":       {"LnormMin": 0,    "LrawMin": 1601, "LnormMax": 100, "LrawMax": 3100}
}

SMS_STS_TORQUE_ENABLE = 40
SMS_STS_GOAL_POSITION_L = 42
SMS_STS_PRESENT_POSITION_L = 56
READ_LEN = 15
CURRENT_LSB_MA = 6.5

INST_WRITE = 0x03
INST_READ = 0x02
INST_SYNC_WRITE = 0x83
BROADCAST_ID = 0xFE

BUS_FAULT_AFTER = 3


# ======================= CARGA DE CALIBRACIÓN =======================
def load_calibration_ini(path):
    """Carga las calibraciones del líder y seguidor desde un archivo .ini dedicado."""
    global CALIBRATION_LEADER, CALIBRATION_FOLLOWER
    if not os.path.isfile(path):
        return

    cp = configparser.ConfigParser(interpolation=None)
    try:
        cp.read(path, encoding="utf-8")
    except Exception:
        return

    def parse_section(section_name, target_dict):
        if not cp.has_section(section_name):
            return
        for joint in JOINTS:
            if cp.has_option(section_name, joint):
                raw_str = cp.get(section_name, joint).strip()
                parts = [p.strip() for p in raw_str.split(",") if p.strip()]
                if len(parts) == 4:
                    try:
                        target_dict[joint] = {
                            "LnormMin": float(parts[0]),
                            "LrawMin": float(parts[1]),
                            "LnormMax": float(parts[2]),
                            "LrawMax": float(parts[3])
                        }
                    except ValueError:
                        pass

    parse_section("leader_calibration", CALIBRATION_LEADER)
    parse_section("follower_calibration", CALIBRATION_FOLLOWER)


# ======================= CONFIGURACIÓN GENERAL =======================
def _to_bool(text):
    return text.strip().lower() in ("1", "true", "yes", "si", "sí", "on")


def load_config(path):
    if not os.path.isfile(path):
        sys.exit(f"No se encuentra el archivo de configuración: {path}\n"
                 f"Crea uno o indica otro con --config.")

    cp = configparser.ConfigParser(interpolation=None)
    try:
        cp.read(path, encoding="utf-8")
    except configparser.Error as e:
        sys.exit(f"Error leyendo {path}: {e}")

    def get(section, key, cast=str, default=None):
        raw = os.getenv(key)
        if raw is None or raw.strip() == "":
            raw = cp.get(section, key, fallback=None)
        if raw is None or raw.strip() == "":
            if default is None:
                sys.exit(f"Falta '{key}' en la sección [{section}] de {path}.")
            return default
        raw = raw.strip()
        try:
            return cast(raw)
        except ValueError:
            sys.exit(f"Valor inválido para {key}: {raw!r}")

    cfg = SimpleNamespace(
        path=path,
        mqtt_broker=get("mqtt", "MQTT_BROKER"),
        mqtt_port=get("mqtt", "MQTT_PORT", int),
        mqtt_user=get("mqtt", "MQTT_USER"),
        mqtt_password=get("mqtt", "MQTT_PASSWORD"),
        leader_port=get("serial", "LEADER_PORT"),
        follower_port=get("serial", "FOLLOWER_PORT"),
        baudrate=get("serial", "BAUDRATE", int),
        mode_source=get("daemon", "MODE_SOURCE", lambda t: t.lower(), default="ctl"),
        so101ctl=get("daemon", "SO101CTL", default="so101ctl"),
        daemon_poll_s=get("daemon", "DAEMON_POLL_S", float, default=0.5),
        daemon_control=get("daemon", "DAEMON_CONTROL", _to_bool, default=False),
        # El dashboard necesita la posición del líder (modo Local: sliders y modelo siguen al líder)
        read_leader=get("serial", "READ_LEADER", _to_bool, default=True),
        # Periodo de publicación de so101/state (0.1 = 10 Hz, para seguimiento en tiempo real)
        state_period_s=get("mqtt", "STATE_PERIOD_S", float, default=0.25),
    )

    if cfg.mode_source == "mqtt":
        sys.exit("MODE_SOURCE=mqtt fue eliminado: el demonio no publica por MQTT. Usa MODE_SOURCE = ctl.")
    if cfg.mode_source not in MODE_SOURCES:
        sys.exit(f"MODE_SOURCE debe ser uno de {', '.join(MODE_SOURCES)}.")
    if cfg.daemon_control and cfg.mode_source != "ctl":
        sys.exit("DAEMON_CONTROL = 1 requiere MODE_SOURCE = ctl (para que el dashboard vea el modo real).")
    if cfg.mode_source == "ctl" or cfg.daemon_control:
        resolved = shutil.which(cfg.so101ctl)
        if not resolved:
            sys.exit(f"No se encuentra so101ctl ('{cfg.so101ctl}'). "
                     f"Indica la ruta completa en SO101CTL, sección [daemon].")
        cfg.so101ctl = resolved
    cfg.daemon_poll_s = max(0.1, cfg.daemon_poll_s)
    cfg.state_period_s = max(0.05, cfg.state_period_s)
    return cfg


# ======================= UTILIDADES DE CINEMÁTICA =======================
def norm_to_raw(joint, norm):
    """Convierte Lnorm a Lraw usando la calibración del LÍDER."""
    if joint not in CALIBRATION_LEADER:
        raise ValueError(f"Articulación sin calibración de líder: {joint}")

    try:
        norm = float(norm)
    except (TypeError, ValueError):
        raise ValueError(f"Lnorm no numérico para {joint}: {norm!r}")

    if not math.isfinite(norm):
        raise ValueError(f"Lnorm no finito para {joint}: {norm!r}")

    cal = CALIBRATION_LEADER[joint]
    norm = max(cal["LnormMin"], min(cal["LnormMax"], norm))

    span_norm = cal["LnormMax"] - cal["LnormMin"]
    span_raw = cal["LrawMax"] - cal["LrawMin"]
    raw = cal["LrawMin"] + ((norm - cal["LnormMin"]) / span_norm) * span_raw

    return int(round(raw))


def raw_to_norm_follower(joint, raw):
    """Convierte Lraw a Lnorm usando la calibración del SEGUIDOR (FOLLOWER)."""
    if joint not in CALIBRATION_FOLLOWER or raw is None:
        return None

    try:
        raw = float(raw)
    except (TypeError, ValueError):
        return None

    if not math.isfinite(raw):
        return None

    cal = CALIBRATION_FOLLOWER[joint]
    raw = max(cal["LrawMin"], min(cal["LrawMax"], raw))

    span_raw = cal["LrawMax"] - cal["LrawMin"]
    span_norm = cal["LnormMax"] - cal["LnormMin"]
    norm = cal["LnormMin"] + ((raw - cal["LrawMin"]) / span_raw) * span_norm

    return round(norm, 1)


def validate_raw(joint, raw_value):
    """Valida una consigna Lraw para el LÍDER con tolerancia de redondeo."""
    if joint not in CALIBRATION_LEADER:
        raise ValueError(f"Articulación sin calibración de líder: {joint}")

    if isinstance(raw_value, bool):
        raise ValueError(f"Lraw inválido para {joint}: {raw_value!r}")

    try:
        raw_float = float(raw_value)
    except (TypeError, ValueError):
        raise ValueError(f"Lraw no numérico para {joint}: {raw_value!r}")

    cal = CALIBRATION_LEADER[joint]
    raw = int(round(raw_float))

    min_allowed = cal["LrawMin"] - 2
    max_allowed = cal["LrawMax"] + 2

    if not min_allowed <= raw <= max_allowed:
        raise ValueError(
            f"Lraw fuera del rango calibrado para {joint}: {raw}; "
            f"rango permitido {cal['LrawMin']}..{cal['LrawMax']}"
        )

    return max(cal["LrawMin"], min(cal["LrawMax"], raw))


def _as_mode(value):
    if isinstance(value, str) and value.strip().lower() in MODES:
        return value.strip().lower()
    return None


def extract_mode_command(text):
    text = text.strip()
    try:
        data = json.loads(text)
    except ValueError:
        data = text
    if isinstance(data, str):
        return _as_mode(data)
    if isinstance(data, dict):
        for key in ("command", "cmd", "action"):
            if key in data:
                return _as_mode(data[key])
        return _as_mode(data.get("mode"))
    return None


# ======================= DEMONIO (so101ctl) =======================
# Formato EXACTO que imprime "so101ctl status" (ver so101ctl.c, función show()):
#   so101d pid N  cycle N  mode <modo>[ (soft start)]  torque ON|off  leader ok|ERR  follower ok|ERR
#   ...
#   faults: none            (o nombres separados por espacios: LEADER_COMM OVERTEMP ...)
_STATUS_RE = re.compile(
    r"mode\s+(?P<mode>\S+)(?P<ramp>\s+\(soft start\))?\s+torque\s+(?P<torque>ON|off)"
    r"\s+leader\s+(?P<leader>ok|ERR)\s+follower\s+(?P<follower>ok|ERR)")
_FAULTS_RE = re.compile(r"^faults:[ \t]*(.*)$", re.MULTILINE)


def parse_ctl_status(text):
    """Interpreta la salida de 'so101ctl status'. Devuelve dict o None si el formato no coincide."""
    m = _STATUS_RE.search(text)
    if not m:
        return None
    fm = _FAULTS_RE.search(text)
    faults_txt = fm.group(1).strip() if fm else ""
    faults = [] if faults_txt.lower() in ("", "none") else faults_txt.split()
    return {
        "mode": m.group("mode").strip().lower(),
        "ramping": bool(m.group("ramp")),
        "torque": m.group("torque") == "ON",
        "leader_ok": m.group("leader") == "ok",
        "follower_ok": m.group("follower") == "ok",
        "faults": faults,
    }


def run_ctl(ctl, args, timeout=2.0):
    """Ejecuta so101ctl. Devuelve (ok, texto): stdout si ok; si no, el mensaje de error."""
    try:
        r = subprocess.run([ctl] + list(args), capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return False, "so101ctl no respondió a tiempo"
    except OSError as e:
        return False, f"no se pudo ejecutar so101ctl: {e}"
    out = (r.stdout or "").strip()
    err = (r.stderr or "").strip()
    if r.returncode == 0:
        return True, out
    return False, err or out or f"so101ctl terminó con código {r.returncode}"


# ======================= DRIVER SERIE =======================
class SO101SerialDriver:
    def __init__(self, name, port, baudrate):
        self.name = name
        self.port = port
        self.baudrate = baudrate
        self.lock = threading.Lock()
        self.ser = None
        self._last_try = 0.0
        self._warned = {}
        self._connect()

    def _warn(self, key, msg, every=30.0):
        """Registra un aviso como mucho una vez cada 'every' segundos por tipo."""
        now = time.time()
        if now - self._warned.get(key, 0.0) >= every:
            self._warned[key] = now
            logger.warning(msg)

    def _connect(self):
        self._last_try = time.time()
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.04)
            logger.info(f"[{self.name}] Conexión serial abierta en {self.port} a {self.baudrate} baudios.")
        except Exception as e:
            self.ser = None
            self._warn("connect", f"[{self.name}] Fallo al abrir puerto serie {self.port}: {e}")

    def _ensure_open(self):
        if self.ser and self.ser.is_open:
            return True
        if time.time() - self._last_try > 2.0:
            self._connect()
        return bool(self.ser and self.ser.is_open)

    @staticmethod
    def _checksum(body):
        return (~sum(body)) & 0xFF

    def _build_packet(self, servo_id, inst, params):
        length = len(params) + 2
        body = [servo_id, length, inst] + list(params)
        return bytes([0xFF, 0xFF] + body + [self._checksum(body)])

    def _send(self, packet):
        with self.lock:
            try:
                if not self._ensure_open():
                    return False
                self.ser.write(packet)
                return True
            except Exception as e:
                self._warn("send", f"[{self.name}] Error escribiendo en el bus: {e}", every=5.0)
                return False

    def sync_write_positions(self, targets, speed=2400):
        if not targets or not self._ensure_open():
            return False

        ln = 6
        params = [SMS_STS_GOAL_POSITION_L, ln]
        for sid, ticks in targets.items():
            ticks = max(0, min(4095, int(ticks)))
            params += [sid, ticks & 0xFF, (ticks >> 8) & 0xFF,
                       0, 0, speed & 0xFF, (speed >> 8) & 0xFF]

        return self._send(self._build_packet(BROADCAST_ID, INST_SYNC_WRITE, params))

    def set_torque(self, enable):
        if not self._ensure_open():
            return False
        params = [SMS_STS_TORQUE_ENABLE, 1 if enable else 0]
        return self._send(self._build_packet(BROADCAST_ID, INST_WRITE, params))

    def read_telemetry(self, servo_id):
        if not self._ensure_open():
            return None

        packet = self._build_packet(servo_id, INST_READ, [SMS_STS_PRESENT_POSITION_L, READ_LEN])
        expected = 6 + READ_LEN

        with self.lock:
            try:
                self.ser.reset_input_buffer()
                self.ser.write(packet)
                response = self.ser.read(expected)
            except Exception:
                return None

        if len(response) < expected or response[0] != 0xFF or response[1] != 0xFF:
            return None
        if response[2] != servo_id or response[3] != READ_LEN + 2:
            return None
        if self._checksum(list(response[2:expected - 1])) != response[expected - 1]:
            self._warn(f"chk{servo_id}", f"[{self.name}] Checksum inválido en respuesta del servo {servo_id}", every=10.0)
            return None
        if response[4] != 0:
            # El servo respondió: sus datos son válidos aunque reporte un flag de error.
            self._warn(f"err{servo_id}", f"[{self.name}] Servo {servo_id} reporta error 0x{response[4]:02X}", every=10.0)

        d = response[5:5 + READ_LEN]
        current_raw = (d[13] | (d[14] << 8)) & 0x7FFF
        return {
            "position": d[0] | (d[1] << 8),
            "voltage": d[6] / 10.0,
            "temperature": d[7],
            "current": round(current_raw * CURRENT_LSB_MA / 1000.0, 3)
        }


# ======================= PUENTE =======================
class SO101Bridge:
    def __init__(self, cfg):
        self.cfg = cfg
        self.leader = SO101SerialDriver("LÍDER", cfg.leader_port, cfg.baudrate)
        self.follower = SO101SerialDriver("SEGUIDOR", cfg.follower_port, cfg.baudrate)

        self.client = mqtt.Client(CallbackAPIVersion.VERSION2, client_id="so101_raspberry_pi")
        self.client.username_pw_set(cfg.mqtt_user, cfg.mqtt_password)
        self.client.will_set(TOPIC_EVENT, json.dumps({"event": "OFFLINE"}), qos=1)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

        self.running = False
        self.emergency = False
        self.control_mode = "local"     # por defecto: el usuario controla el líder a mano
        self._lock = threading.Lock()

        self.mode = "unknown"
        self.mode_origin = "none"
        self.daemon_faults = []
        self.daemon_online = None      # None = aún no se sabe
        self.daemon_info = None
        self._last_faults = {}

        self.real_position = {j: None for j in JOINTS}
        self.voltage = {j: None for j in JOINTS}
        self.temperature = {j: None for j in JOINTS}
        self.current = {j: None for j in JOINTS}
        self.leader_position = {j: None for j in JOINTS}
        self.fail_count = {j: 0 for j in JOINTS}

    def _publish_result(self, status, detail):
        self.client.publish(TOPIC_RESULT, json.dumps({"status": status, "detail": detail}), qos=1)

    def _publish_event(self, name, data=None):
        event = {"event": name}
        if data:
            event.update(data)
        event["timestamp"] = time.time()
        self.client.publish(TOPIC_EVENT, json.dumps(event), qos=1)

    def _set_mode(self, new_mode, origin):
        with self._lock:
            old = self.mode
            if new_mode == old:
                return
            self.mode = new_mode
            self.mode_origin = origin
        logger.info(f"Modo: {old} -> {new_mode} (origen: {origin})")
        self._publish_event("MODE_CHANGED", {"from": old, "to": new_mode, "source": origin})
        if new_mode == "estop":
            self._apply_control("local", "daemon")

    def on_connect(self, client, userdata, flags, rc, properties=None):
        if rc == 0:
            logger.info(f"Conectado exitosamente al broker MQTT como '{self.cfg.mqtt_user}'.")
            client.subscribe(TOPIC_CMD)
            self._publish_event("ONLINE", {"mode": self.mode, "control_mode": self.control_mode})
        else:
            logger.error(f"Fallo de conexión MQTT (rc={rc}).")

    def on_disconnect(self, client, userdata, flags, rc, properties=None):
        if rc != 0:
            logger.warning(f"Desconectado inesperadamente del broker MQTT (rc={rc}).")
        else:
            logger.info("Desconectado del broker MQTT.")

    def on_message(self, client, userdata, msg):
        try:
            text = msg.payload.decode("utf-8")
        except UnicodeDecodeError:
            return
        try:
            if msg.topic == TOPIC_CMD:
                self._handle_cmd(text)
        except Exception as e:
            logger.error(f"Error procesando mensaje en {msg.topic}: {e}")
            if msg.topic == TOPIC_CMD:
                self._publish_result("BAD_CMD", f"Error: {e}")

    def _handle_cmd(self, text):
        # Formas antiguas de pedir un modo: texto plano ("teleop") o JSON {"mode": ...} / {"command": "teleop"}
        requested = extract_mode_command(text)
        if requested:
            self._request_mode(requested)
            return

        try:
            payload = json.loads(text)
        except ValueError:
            self._publish_result("BAD_CMD", "El mensaje no es JSON ni un comando de modo.")
            return
        if not isinstance(payload, dict):
            self._publish_result("BAD_CMD", "El mensaje debe ser un objeto JSON.")
            return

        cmd = payload.get("command")

        if not cmd:
            self._publish_result("BAD_CMD", "Falta el atributo 'command'")
            return

        elif cmd == "MOVE_JOINTS":
            logger.info("Comando recibido: MOVE_JOINTS")

            if self.emergency:
                self._publish_result("REJECTED", "Parada de emergencia activa.")
                return
            if self.mode == "estop":
                self._publish_result("REJECTED", "El demonio está en modo estop.")
                return
            if self.control_mode != "dashboard":
                self._publish_result(
                    "REJECTED", "Modo Local: el líder lo mueve el usuario; el puente no acepta consignas.")
                return

            use_raw = "target_raw" in payload

            if use_raw:
                incoming = payload.get("target_raw")
                field_name = "target_raw"
                if not isinstance(incoming, dict):
                    self._publish_result("BAD_CMD", "'target_raw' debe ser un objeto.")
                    return
            else:
                incoming = payload.get("target_position", {})
                field_name = "target_position"
                if not isinstance(incoming, dict):
                    self._publish_result("BAD_CMD", "'target_position' debe ser un objeto.")
                    return

            targets = {}
            names = []
            unknown = []

            for joint, value in incoming.items():
                if joint not in JOINTS:
                    unknown.append(str(joint))
                    continue

                try:
                    if use_raw:
                        raw = validate_raw(joint, value)
                    else:
                        raw = norm_to_raw(joint, value)
                except ValueError as e:
                    self._publish_result("BAD_CMD", str(e))
                    return

                targets[JOINTS[joint]] = raw
                names.append(joint)

            if not targets:
                self._publish_result(
                    "BAD_CMD",
                    f"No se encontraron articulaciones válidas en '{field_name}'."
                )
                return

            if self.leader.sync_write_positions(targets):
                source_desc = "Lraw" if use_raw else "Lnorm convertido a Lraw"
                detail = f"Consigna {source_desc} enviada al LÍDER para: {', '.join(names)}."
                if unknown:
                    detail += f" Ignoradas: {', '.join(unknown)}."
                if not payload.get("stream"):        # consignas en ráfaga (sliders): sin resultado por cada una
                    self._publish_result("OK", detail)
            else:
                self._publish_result(
                    "BUS_ERROR",
                    f"No se pudo escribir en el bus del líder para: {', '.join(names)}"
                )

        elif cmd == "SET_CONTROL":
            control = payload.get("control")
            if control not in CONTROL_MODES:
                self._publish_result(
                    "BAD_CMD", f"Modo de control inválido: {control!r}. Opciones: {', '.join(CONTROL_MODES)}.")
            elif control == "dashboard" and (self.emergency or self.mode == "estop"):
                self._publish_result(
                    "REJECTED", "No se puede pasar a Modo Dashboard con la parada de emergencia activa.")
            else:
                self._apply_control(control, "dashboard")
                self._publish_result("OK", f"Modo de control: {control}.")

        elif cmd == "SET_MODE":
            mode = _as_mode(payload.get("mode"))
            if not mode:
                self._publish_result(
                    "BAD_CMD", f"Modo inválido: {payload.get('mode')!r}. Modos: {', '.join(MODES)}.")
            else:
                self._request_mode(mode)

        elif cmd == "EMERGENCY_STOP":
            self._do_estop()

        elif cmd == "RESET_ESTOP":
            self._do_reset()

        else:
            self._publish_result("BAD_CMD", f"Comando '{cmd}' no soportado.")

    # ---------- modos del demonio (equivale a escribir 'so101ctl <modo>' en la Raspberry) ----------
    def _apply_control(self, new, source):
        """Cambia entre 'local' y 'dashboard'. Devuelve True si hubo cambio."""
        with self._lock:
            old = self.control_mode
            if new == old:
                return False
            self.control_mode = new
        if LEADER_TORQUE:
            # Líder real: torque ON solo en modo dashboard; OFF en modo local para poder moverlo a mano.
            self.leader.set_torque(new == "dashboard")
        logger.info(f"Modo de control: {old} -> {new} (origen: {source})")
        self._publish_event("CONTROL_CHANGED", {"from": old, "to": new, "source": source})
        return True

    def _request_mode(self, mode):
        logger.info(f"Cambio de modo solicitado: {mode}")

        # estop y reset siempre se atienden: activan/quitan la parada del puente y,
        # con DAEMON_CONTROL = 1, también se lo ordenan al demonio.
        if mode == "estop":
            self._do_estop()
            return
        if mode == "reset":
            self._do_reset()
            return

        if not self.cfg.daemon_control:
            if self.cfg.mode_source == "cmd":
                self._set_mode(mode, "cmd")      # solo pruebas sin demonio
            else:
                self._publish_result(
                    "REJECTED", "Control del demonio desactivado (DAEMON_CONTROL = 0 en el .ini).")
            return

        ok, out = run_ctl(self.cfg.so101ctl, [mode])
        self._publish_result("OK" if ok else "DAEMON_ERROR", f"so101ctl {mode}: {out}")

    def _do_estop(self):
        self.emergency = True
        self._apply_control("local", "estop")
        if LEADER_TORQUE:
            self.leader.set_torque(False)
        status = "OK"
        detail = "Parada de emergencia activada (se bloquean nuevos MOVE_JOINTS)."
        if self.cfg.daemon_control:
            ok, out = run_ctl(self.cfg.so101ctl, ["estop"])
            detail += f" Demonio (estop): {out}"
            if not ok:
                status = "DAEMON_ERROR"
        self._publish_result(status, detail)
        self._publish_event("FAULT_TRIGGERED", {"fault": "EMERGENCY_STOP", "source": "bridge"})

    def _do_reset(self):
        if self.cfg.daemon_control:
            ok, out = run_ctl(self.cfg.so101ctl, ["reset"])
            if not ok:
                # Si el demonio no acepta el rearme, la parada del puente se mantiene.
                self._publish_result("DAEMON_ERROR", f"El demonio no aceptó reset: {out}. La parada sigue activa.")
                return
        was_latched = self.emergency
        self.emergency = False
        self._publish_result("OK", "Parada de emergencia desactivada.")
        if was_latched:
            self._publish_event("FAULT_CLEARED", {"fault": "EMERGENCY_STOP", "source": "bridge"})

    def _emit_fault_events(self, current):
        for name, origin in current.items():
            if name not in self._last_faults and name not in EVENT_SKIP_FAULTS:
                self._publish_event("FAULT_TRIGGERED", {"fault": name, "source": origin})
        for name, origin in self._last_faults.items():
            if name not in current and name not in EVENT_SKIP_FAULTS:
                self._publish_event("FAULT_CLEARED", {"fault": name, "source": origin})
        self._last_faults = dict(current)

    def daemon_poll_loop(self):
        """Lee modo y fallas REALES del demonio con 'so101ctl status' (memoria compartida)."""
        fails = 0
        while self.running:
            ok, out = run_ctl(self.cfg.so101ctl, ["status"], timeout=1.5)
            status = parse_ctl_status(out) if ok else None
            if status:
                fails = 0
                with self._lock:
                    self.daemon_online = True
                    self.daemon_info = {k: status[k] for k in ("ramping", "torque", "leader_ok", "follower_ok")}
                    self.daemon_faults = status["faults"]
                self._set_mode(status["mode"], "daemon")
            else:
                fails += 1
                if fails == 1 or fails % 40 == 0:
                    logger.warning("No se pudo leer el estado del demonio: "
                                   + (out if not ok else "formato inesperado en la salida de 'so101ctl status'"))
                if fails >= 2:
                    with self._lock:
                        self.daemon_online = False
                        self.daemon_info = None
                        self.daemon_faults = []
                    if self.mode != "unknown":
                        self._set_mode("unknown", "daemon")
            time.sleep(self.cfg.daemon_poll_s)

    def telemetry_loop(self):
        """Publica en so101/state la posición normalizada (Lnorm) del SEGUIDOR."""
        while self.running:
            telemetry = {}
            faults = {}
            if self.emergency:
                faults["EMERGENCY_STOP_TRIGGERED"] = "bridge"

            for joint_name, servo_id in JOINTS.items():
                data = self.follower.read_telemetry(servo_id)
                if data:
                    self.fail_count[joint_name] = 0
                    self.real_position[joint_name] = raw_to_norm_follower(joint_name, data["position"])
                    self.voltage[joint_name] = data["voltage"]
                    self.temperature[joint_name] = float(data["temperature"])
                    self.current[joint_name] = data["current"]
                else:
                    self.fail_count[joint_name] += 1
                    if self.fail_count[joint_name] >= BUS_FAULT_AFTER:
                        self.real_position[joint_name] = None
                        self.voltage[joint_name] = None
                        self.temperature[joint_name] = None
                        self.current[joint_name] = None
                        faults[f"BUS_TIMEOUT:{joint_name}"] = "bridge"

                telemetry[joint_name] = {
                    "temp": self.temperature[joint_name],
                    "volt": self.voltage[joint_name],
                    "curr": self.current[joint_name],
                }

                if self.cfg.read_leader:
                    ld = self.leader.read_telemetry(servo_id)
                    if ld:
                        cal_l = CALIBRATION_LEADER.get(joint_name)
                        if cal_l:
                            raw_l = max(cal_l["LrawMin"], min(cal_l["LrawMax"], float(ld["position"])))
                            span_r = cal_l["LrawMax"] - cal_l["LrawMin"]
                            span_n = cal_l["LnormMax"] - cal_l["LnormMin"]
                            self.leader_position[joint_name] = round(cal_l["LnormMin"] + ((raw_l - cal_l["LrawMin"]) / span_r) * span_n, 1)
                        else:
                            self.leader_position[joint_name] = None
                    else:
                        self.leader_position[joint_name] = None

            with self._lock:
                daemon_faults = list(self.daemon_faults)
                daemon_online = self.daemon_online
                daemon_info = dict(self.daemon_info) if self.daemon_info else {}
            for name in daemon_faults:
                faults.setdefault(name, "daemon")
            if self.cfg.mode_source == "ctl" and daemon_online is False:
                faults["DAEMON_OFFLINE"] = "bridge"

            self._emit_fault_events(faults)

            with self._lock:
                mode, mode_origin = self.mode, self.mode_origin

            state_packet = {
                "timestamp": int(time.time() * 1000),
                "mode": mode,
                "mode_source": mode_origin,
                "faults": list(faults),
                "control_mode": self.control_mode,    # local | dashboard
                "position": self.real_position,       # SEGUIDOR (Lnorm)
                "telemetry": telemetry
            }
            if self.cfg.mode_source == "ctl":
                state_packet["daemon"] = {"online": daemon_online, **daemon_info}
            if self.cfg.read_leader:
                state_packet["leader_position"] = self.leader_position   # LÍDER (Lnorm)

            self.client.publish(TOPIC_STATE, json.dumps(state_packet), qos=0)
            time.sleep(self.cfg.state_period_s)

    def start(self):
        cfg = self.cfg
        self.running = True
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)
        try:
            while True:
                try:
                    self.client.connect(cfg.mqtt_broker, cfg.mqtt_port, keepalive=60)
                    break
                except OSError as e:
                    logger.error(f"No se pudo conectar al broker MQTT ({cfg.mqtt_broker}:{cfg.mqtt_port}): {e}. Reintentando en 5 s...")
                    time.sleep(5)

            if LEADER_TORQUE:
                self.leader.set_torque(False)     # modo local por defecto: líder libre

            self.client.loop_start()
            threading.Thread(target=self.telemetry_loop, daemon=True).start()
            if cfg.mode_source == "ctl":
                threading.Thread(target=self.daemon_poll_loop, daemon=True).start()
            logger.info(f"Puente en ejecución (config: {cfg.path}) | modo: {cfg.mode_source} | "
                        f"control del demonio: {'ON' if cfg.daemon_control else 'off'} | "
                        f"modo de control: {self.control_mode} | líder: {'leído' if cfg.read_leader else 'NO leído'}. Ctrl+C para salir.")

            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            pass
        finally:
            self.running = False
            if LEADER_TORQUE:
                self.leader.set_torque(False)
            try:
                if self.client.is_connected():
                    info = self.client.publish(TOPIC_EVENT, json.dumps({"event": "OFFLINE"}), qos=1)
                    info.wait_for_publish(timeout=2)
            except Exception:
                pass
            self.client.loop_stop()
            self.client.disconnect()


def main():
    ap = argparse.ArgumentParser(description="Puente MQTT <-> brazo SO101 (líder/seguidor)")
    ap.add_argument("-c", "--config",
                    default=os.getenv("SO101_BRIDGE_INI", DEFAULT_INI),
                    help=f"archivo .ini de configuración general (por defecto: {DEFAULT_INI})")
    ap.add_argument("--calib",
                    default=os.getenv("SO101_CALIB_INI", DEFAULT_CALIB_INI),
                    help=f"archivo .ini de calibración de articulaciones (por defecto: {DEFAULT_CALIB_INI})")
    args = ap.parse_args()

    load_calibration_ini(args.calib)
    SO101Bridge(load_config(args.config)).start()


if __name__ == "__main__":
    main()
