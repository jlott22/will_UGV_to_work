#!/usr/bin/env python3
"""
UGV_Navigation.py

Navigation process for the multi-UGV semantic search project.

Responsibilities:
- Read GPS from serial.
- Read RPLidar scan data from the rplidar_sdk ultra_simple process.
- Send simple movement commands to the Arduino motor controller.
- Receive GPS waypoint commands from TaskManager over local MQTT.
- Drive to ONE waypoint, stop, publish waypoint_reached, then wait for the next waypoint.
- Publish LiDAR adjacent-cell scan results for TaskManager map updates.

Architecture rule:
- Navigation owns GPS, LiDAR thresholding, occupied-cell geometry, and motor commands.
- TaskManager owns the grid, belief map, task allocation, and path planning.
- Navigation does NOT receive interrogation targets, rotate commands, or camera commands.
"""

import json
import math
import os
import select
import signal
import struct
import subprocess
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

try:
    import serial
except ImportError as exc:
    raise SystemExit("Missing dependency: pyserial. Install with: pip3 install pyserial") from exc

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:
    raise SystemExit("Missing dependency: paho-mqtt. Install with: pip3 install paho-mqtt") from exc


# ===========================================================
# USER-EDITABLE CONFIGURATION
# ===========================================================

ROBOT_ID = "00"  # USER EDIT: match TaskManager ROBOT_ID

CELL_SIZE_M = 1.0
# Grid resolution is 1.0 m per cell. TaskManager and Navigation must use the same value.

MQTT_BROKER_HOST = "localhost"
MQTT_BROKER_PORT = 1883
MQTT_KEEPALIVE = 10

TRIAL_ID = time.strftime("%Y%m%d-%H%M%S")
LOG_DIR = "/home/jetson-nano/ugv/logs"
NAV_EVENT_LOG_FILE = "nav-events.jsonl"
NAV_ERROR_LOG_FILE = "nav-errors.jsonl"

GPS_PORT = "/dev/ttyTHS1"
GPS_BAUD = 115200

ARD_PORT = "/dev/ttyUSB1"
ARD_BAUD = 115200

LIDAR_CMD = [
    "/home/jetson-nano/rplidar_sdk/output/Linux/Release/ultra_simple",
    "--channel",
    "--serial",
    "/dev/ttyUSB0",
    "256000",
]

# Arduino command timing. These preserve the burst-style control used by the
# current navigation code, but the destination now comes from TaskManager.
F_BURST_S = 0.55
TURN_BURST_S = 0.35
SMALL_TURN_BURST_S = 0.18
SETTLE_TIME_S = 0.25

ARDUINO_BOOT_WAIT_S = 3.0
ARDUINO_POST_STOP_WAIT_S = 0.5

# GPS navigation tuning.
WAYPOINT_RADIUS_M = 0.25
REQUIRE_RTK_FIXED = True
MAX_H_ACC_M = 0.20
MIN_HEADING_SPEED_MPS = 0.30
INITIAL_HEADING_ENABLED = True
INITIAL_HEADING_MIN_MOVE_M = 1.0
INITIAL_HEADING_FORWARD_CMD = "F"
INITIAL_HEADING_BURST_S = 0.30
INITIAL_HEADING_MAX_ATTEMPTS = 8
INITIAL_HEADING_MAX_TOTAL_S = 20.0
INITIAL_HEADING_MIN_GROUND_SPEED_MPS = 0.05
NO_FIX_SLEEP_S = 0.5
IDLE_SLEEP_S = 0.1

BIG_ERR_DEG = 45.0
SMALL_ERR_DEG = 18.0

# LiDAR scan behavior.
# The TaskManager assumes Navigation owns LiDAR geometry and thresholding.
LIDAR_WINDOW_SECONDS = 0.18
LIDAR_OCCUPIED_THRESHOLD_M = CELL_SIZE_M
LIDAR_HARD_STOP_M = 0.30
LIDAR_MIN_VALID_M = 0.05
LIDAR_MAX_REPORT_M = 2.0

# If the LiDAR's 0-degree direction is not physically aligned with the car's
# forward direction, adjust this offset. Current code assumes 0 = forward.
LIDAR_YAW_OFFSET_DEG = 0.0

# Adjacent LiDAR scans are only published at waypoint arrival for this phase.
STATUS_PUBLISH_INTERVAL_S = 0.8


# ===========================================================
# MQTT TOPICS - must match UGV_TaskManager.py
# ===========================================================

TOPIC_CMD_WAYPOINT = f"/ugv/{ROBOT_ID}/cmd/waypoint"
TOPIC_CMD_STOP = f"/ugv/{ROBOT_ID}/cmd/stop"

TOPIC_NAV_STARTUP = f"/ugv/{ROBOT_ID}/nav/startup"
TOPIC_NAV_STATUS = f"/ugv/{ROBOT_ID}/nav/status"
TOPIC_NAV_ADJACENT_SCAN = f"/ugv/{ROBOT_ID}/nav/adjacent_scan"


# ===========================================================
# Persistent JSONL logging
# ===========================================================

def trial_log_filename(base_name: str) -> str:
    root, ext = os.path.splitext(base_name)
    filename = f"{root}-{TRIAL_ID}{ext or '.jsonl'}"
    return os.path.join(LOG_DIR, filename)


NAV_EVENT_LOG_PATH = trial_log_filename(NAV_EVENT_LOG_FILE)
NAV_ERROR_LOG_PATH = trial_log_filename(NAV_ERROR_LOG_FILE)


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def write_jsonl(path: str, entry: Dict[str, Any]):
    try:
        os.makedirs(LOG_DIR, exist_ok=True)
        with open(path, "a", encoding="utf-8") as fh:
            fh.write(json.dumps(json_safe(entry), separators=(",", ":")) + "\n")
            fh.flush()
    except Exception as exc:
        try:
            print(f"{ts()} WARNING failed writing log {path}: {exc}")
        except Exception:
            pass


def log_nav_event(event_type: str, fields: Optional[Dict[str, Any]] = None):
    write_jsonl(
        NAV_EVENT_LOG_PATH,
        {
            "timestamp": time.time(),
            "trial_id": TRIAL_ID,
            "robot_id": ROBOT_ID,
            "event_type": event_type,
            "fields": fields or {},
        },
    )


def log_nav_error(event_type: str, exc_or_message, fields: Optional[Dict[str, Any]] = None):
    write_jsonl(
        NAV_ERROR_LOG_PATH,
        {
            "timestamp": time.time(),
            "trial_id": TRIAL_ID,
            "robot_id": ROBOT_ID,
            "event_type": event_type,
            "error": repr(exc_or_message),
            "fields": fields or {},
        },
    )


def nav_program_start_fields() -> Dict[str, Any]:
    return {
        "trial_id": TRIAL_ID,
        "robot_id": ROBOT_ID,
        "script_name": os.path.basename(__file__),
        "timestamp": time.time(),
        "current_working_directory": os.getcwd(),
        "event_log": NAV_EVENT_LOG_PATH,
        "error_log": NAV_ERROR_LOG_PATH,
        "key_config": {
            "log_dir": LOG_DIR,
            "mqtt_broker_host": MQTT_BROKER_HOST,
            "mqtt_broker_port": MQTT_BROKER_PORT,
            "gps_port": GPS_PORT,
            "gps_baud": GPS_BAUD,
            "arduino_port": ARD_PORT,
            "arduino_baud": ARD_BAUD,
            "cell_size_m": CELL_SIZE_M,
            "waypoint_radius_m": WAYPOINT_RADIUS_M,
            "require_rtk_fixed": REQUIRE_RTK_FIXED,
            "max_h_acc_m": MAX_H_ACC_M,
            "min_heading_speed_mps": MIN_HEADING_SPEED_MPS,
            "initial_heading_enabled": INITIAL_HEADING_ENABLED,
            "initial_heading_min_move_m": INITIAL_HEADING_MIN_MOVE_M,
            "lidar_occupied_threshold_m": LIDAR_OCCUPIED_THRESHOLD_M,
            "lidar_hard_stop_m": LIDAR_HARD_STOP_M,
        },
    }


def nav_program_shutdown_fields(reason: str) -> Dict[str, Any]:
    return {
        "trial_id": TRIAL_ID,
        "robot_id": ROBOT_ID,
        "script_name": os.path.basename(__file__),
        "timestamp": time.time(),
        "reason": reason,
    }


# ===========================================================
# Data structures
# ===========================================================

GPS = Tuple[float, float]


@dataclass
class Waypoint:
    lat: float
    lon: float
    cell_x: Optional[int] = None
    cell_y: Optional[int] = None
    timestamp: float = 0.0


@dataclass
class Fix:
    lat: float
    lon: float
    heading_deg: float
    h_acc_m: float
    fix_type: int
    carr_soln: int
    rtk_fixed: bool
    diff_soln: bool
    num_sv: int
    ground_speed_mps: float
    timestamp: float
    gnss_fix_ok: bool = False


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.active_waypoint: Optional[Waypoint] = None
        self.stop_requested = False
        self.last_command_time = 0.0

    def set_waypoint(self, wp: Waypoint):
        with self.lock:
            self.active_waypoint = wp
            self.stop_requested = False
            self.last_command_time = time.time()

    def clear_waypoint(self):
        with self.lock:
            self.active_waypoint = None

    def request_stop(self):
        with self.lock:
            self.stop_requested = True
            self.active_waypoint = None
            self.last_command_time = time.time()

    def snapshot(self) -> Tuple[Optional[Waypoint], bool]:
        with self.lock:
            return self.active_waypoint, self.stop_requested


# ===========================================================
# Utility functions
# ===========================================================

EARTH_RADIUS_M = 6371000.0


def ts() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M:%S")


def normalize_angle_deg(a: float) -> float:
    while a > 180.0:
        a -= 360.0
    while a < -180.0:
        a += 360.0
    return a


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    b = math.degrees(math.atan2(y, x))
    return (b + 360.0) % 360.0


def project_gps(lat: float, lon: float, bearing: float, distance_m: float) -> GPS:
    brng = math.radians(bearing)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    dr = distance_m / EARTH_RADIUS_M

    lat2 = math.asin(
        math.sin(lat1) * math.cos(dr)
        + math.cos(lat1) * math.sin(dr) * math.cos(brng)
    )
    lon2 = lon1 + math.atan2(
        math.sin(brng) * math.sin(dr) * math.cos(lat1),
        math.cos(dr) - math.sin(lat1) * math.sin(lat2),
    )
    return math.degrees(lat2), math.degrees(lon2)


def open_serial_with_retry(port: str, baud: int, name: str) -> serial.Serial:
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print(f"{ts()} Opened {name} on {port} @ {baud}")
            log_nav_event("serial_open_success", {"name": name, "port": port, "baud": baud})
            return ser
        except Exception as exc:
            print(f"{ts()} Failed opening {name} on {port}: {exc}")
            log_nav_error("serial_open_failure", exc, {"name": name, "port": port, "baud": baud})
            time.sleep(1.0)


def send_cmd(ser: serial.Serial, cmd: str):
    log_nav_event("motor_command", {"cmd": cmd})
    try:
        ser.write(cmd.encode("ascii"))
        ser.flush()
    except Exception as exc:
        log_nav_error("serial_write_failure", exc, {"cmd": cmd})
        raise


def stop_motors(ard: serial.Serial):
    try:
        send_cmd(ard, "S")
    except Exception as exc:
        print(f"{ts()} Failed to send stop command: {exc}")
        log_nav_error("motor_command_failed", exc, {"cmd": "S"})


UBX_SYNC_1 = 0xB5
UBX_SYNC_2 = 0x62
UBX_CLASS_NAV = 0x01
UBX_ID_NAV_PVT = 0x07
UBX_NAV_PVT_LEN = 92


def ubx_checksum(packet_body: bytes) -> Tuple[int, int]:
    ck_a = 0
    ck_b = 0
    for byte in packet_body:
        ck_a = (ck_a + byte) & 0xFF
        ck_b = (ck_b + ck_a) & 0xFF
    return ck_a, ck_b


def read_exact(ser: serial.Serial, size: int, deadline: float) -> Optional[bytes]:
    data = bytearray()
    while len(data) < size and time.time() < deadline:
        chunk = ser.read(size - len(data))
        if chunk:
            data.extend(chunk)
    if len(data) != size:
        return None
    return bytes(data)


def parse_nav_pvt(payload: bytes) -> Fix:
    fix_type = payload[20]
    flags = payload[21]
    num_sv = payload[23]
    lon_raw = struct.unpack_from("<i", payload, 24)[0]
    lat_raw = struct.unpack_from("<i", payload, 28)[0]
    h_acc_raw = struct.unpack_from("<I", payload, 40)[0]
    g_speed_raw = struct.unpack_from("<i", payload, 60)[0]
    head_mot_raw = struct.unpack_from("<i", payload, 64)[0]

    gnss_fix_ok = bool(flags & 0x01)
    diff_soln = bool(flags & 0x02)
    carr_soln = (flags >> 6) & 0x03
    heading_deg = (head_mot_raw * 1e-5) % 360.0

    return Fix(
        lat=lat_raw * 1e-7,
        lon=lon_raw * 1e-7,
        heading_deg=heading_deg,
        h_acc_m=h_acc_raw / 1000.0,
        fix_type=fix_type,
        carr_soln=carr_soln,
        rtk_fixed=(carr_soln == 2),
        diff_soln=diff_soln,
        num_sv=num_sv,
        ground_speed_mps=g_speed_raw / 1000.0,
        timestamp=time.time(),
        gnss_fix_ok=gnss_fix_ok,
    )


def read_fix(gps: serial.Serial, timeout_s: float = 1.5) -> Optional[Fix]:
    """Read and validate one UBX NAV-PVT packet from the ZED-F9R."""
    end = time.time() + timeout_s
    while time.time() < end:
        first = gps.read(1)
        if not first or first[0] != UBX_SYNC_1:
            continue

        second = read_exact(gps, 1, end)
        if second is None or second[0] != UBX_SYNC_2:
            continue

        header = read_exact(gps, 4, end)
        if header is None:
            return None

        msg_class = header[0]
        msg_id = header[1]
        payload_len = struct.unpack_from("<H", header, 2)[0]
        payload_and_checksum = read_exact(gps, payload_len + 2, end)
        if payload_and_checksum is None:
            return None

        payload = payload_and_checksum[:payload_len]
        ck_a_rx = payload_and_checksum[payload_len]
        ck_b_rx = payload_and_checksum[payload_len + 1]

        if (
            msg_class != UBX_CLASS_NAV
            or msg_id != UBX_ID_NAV_PVT
            or payload_len != UBX_NAV_PVT_LEN
        ):
            continue

        ck_a, ck_b = ubx_checksum(header + payload)
        if ck_a != ck_a_rx or ck_b != ck_b_rx:
            print(f"{ts()} Ignoring NAV-PVT with bad checksum")
            continue

        return parse_nav_pvt(payload)
    return None


def fix_is_navigation_usable(fix: Optional[Fix]) -> bool:
    if fix is None:
        return False
    if fix.fix_type < 3:
        return False
    if not fix.gnss_fix_ok:
        return False
    if fix.carr_soln != 2:
        return False
    return fix.h_acc_m <= MAX_H_ACC_M


def filtered_heading_from_fix(
    fix: Fix,
    last_valid_heading_deg: Optional[float],
) -> Tuple[Optional[float], Optional[float], bool]:
    """
    Return (heading_to_use, updated_last_valid_heading_deg, heading_updated).

    Use fix.heading_deg only when fix.ground_speed_mps >= MIN_HEADING_SPEED_MPS.
    Otherwise hold last_valid_heading_deg.
    If no previous heading exists and speed is below threshold, return None.
    """
    if fix.ground_speed_mps >= MIN_HEADING_SPEED_MPS:
        heading = float(fix.heading_deg) % 360.0
        return heading, heading, True

    return last_valid_heading_deg, last_valid_heading_deg, False


def read_usable_fix(gps: serial.Serial, timeout_s: float) -> Optional[Fix]:
    end = time.time() + timeout_s
    while time.time() < end:
        fix = read_fix(gps, timeout_s=min(1.0, max(0.05, end - time.time())))
        if fix_is_navigation_usable(fix):
            return fix
    return None


def acquire_initial_heading(
    gps: serial.Serial,
    ard: serial.Serial,
    nav: "NavigationNode",
    running_check,
) -> Optional[Tuple[Fix, float]]:
    print(f"{ts()} Starting heading acquisition")
    log_nav_event("heading_acquisition_start", {})
    started_at = time.time()
    last_status_publish = 0.0
    start_fix: Optional[Fix] = None
    latest_fix: Optional[Fix] = None
    original_gps_timeout = getattr(gps, "timeout", None)

    try:
        gps.timeout = 0.05
        while running_check() and time.time() - started_at < INITIAL_HEADING_MAX_TOTAL_S:
            start_fix = read_usable_fix(gps, timeout_s=1.0)
            if start_fix is not None:
                latest_fix = start_fix
                break

            now = time.time()
            if latest_fix is not None and now - last_status_publish >= STATUS_PUBLISH_INTERVAL_S:
                nav.publish_status(latest_fix, None, "no_heading")
                last_status_publish = now
            stop_motors(ard)

        if start_fix is None:
            print(f"{ts()} Heading acquisition failed")
            log_nav_event("heading_acquisition_failed", {"reason": "no_usable_start_fix"})
            if latest_fix is not None:
                nav.publish_status(latest_fix, None, "heading_acquisition_failed")
            stop_motors(ard)
            return None

        for attempt in range(1, INITIAL_HEADING_MAX_ATTEMPTS + 1):
            if not running_check() or time.time() - started_at >= INITIAL_HEADING_MAX_TOTAL_S:
                break

            print(f"{ts()} Heading acquisition burst {attempt}/{INITIAL_HEADING_MAX_ATTEMPTS}")
            log_nav_event("heading_acquisition_burst", {"attempt": attempt})
            send_cmd(ard, INITIAL_HEADING_FORWARD_CMD)
            burst_end = time.time() + INITIAL_HEADING_BURST_S
            fix: Optional[Fix] = None
            while running_check() and time.time() < burst_end:
                candidate = read_usable_fix(gps, timeout_s=min(0.2, max(0.05, burst_end - time.time())))
                if candidate is not None:
                    fix = candidate
            stop_motors(ard)
            time.sleep(SETTLE_TIME_S)

            if fix is None:
                fix = read_usable_fix(gps, timeout_s=1.5)
            if fix is None:
                now = time.time()
                if latest_fix is not None and now - last_status_publish >= STATUS_PUBLISH_INTERVAL_S:
                    nav.publish_status(latest_fix, None, "no_heading")
                    last_status_publish = now
                continue

            latest_fix = fix
            moved_m = haversine_m(start_fix.lat, start_fix.lon, fix.lat, fix.lon)
            print(f"{ts()} Heading acquisition moved {moved_m:.2f} m")
            log_nav_event(
                "heading_acquisition_moved",
                {
                    "attempt": attempt,
                    "moved_m": moved_m,
                    "start_lat": start_fix.lat,
                    "start_lon": start_fix.lon,
                    "current_lat": fix.lat,
                    "current_lon": fix.lon,
                    "current_heading_deg": fix.heading_deg,
                    "ground_speed_mps": fix.ground_speed_mps,
                },
            )

            if moved_m < INITIAL_HEADING_MIN_MOVE_M:
                now = time.time()
                if now - last_status_publish >= STATUS_PUBLISH_INTERVAL_S:
                    nav.publish_status(fix, None, "no_heading")
                    last_status_publish = now
                continue

            gps_bearing = bearing_deg(start_fix.lat, start_fix.lon, fix.lat, fix.lon)
            nav_pvt_heading: Optional[float] = None
            if fix.ground_speed_mps >= INITIAL_HEADING_MIN_GROUND_SPEED_MPS:
                nav_pvt_heading = float(fix.heading_deg) % 360.0
            chosen_heading = nav_pvt_heading if nav_pvt_heading is not None else gps_bearing
            print(
                f"{ts()} Heading acquired: "
                f"nav_pvt={nav_pvt_heading if nav_pvt_heading is not None else 'None'}, "
                f"gps_bearing={gps_bearing:.1f}"
            )
            log_nav_event(
                "heading_acquired",
                {
                    "nav_pvt_heading_deg": nav_pvt_heading,
                    "gps_bearing_deg": gps_bearing,
                    "chosen_heading_deg": chosen_heading,
                    "moved_m": moved_m,
                    "ground_speed_mps": fix.ground_speed_mps,
                },
            )
            stop_motors(ard)
            return fix, chosen_heading

        print(f"{ts()} Heading acquisition failed")
        log_nav_event("heading_acquisition_failed", {"reason": "attempt_or_timeout_limit"})
        if latest_fix is not None:
            nav.publish_status(latest_fix, None, "heading_acquisition_failed")
        stop_motors(ard)
        return None
    except Exception:
        stop_motors(ard)
        raise
    finally:
        gps.timeout = original_gps_timeout


def gps_quality_alarm():
    """Placeholder for a buzzer or external alert when RTK fixed navigation is lost."""
    pass


def print_fix_debug(fix: Fix, rtk_state: str):
    print(
        f"{ts()} GPS lat={fix.lat:.7f} lon={fix.lon:.7f} "
        f"heading={fix.heading_deg:.1f}deg h_acc={fix.h_acc_m:.3f}m "
        f"carrSoln={fix.carr_soln} num_sv={fix.num_sv} RTK={rtk_state}"
    )


def do_burst(ard: serial.Serial, cmd: str, duration_s: float):
    print(f"{ts()} BURST {cmd} for {duration_s:.2f}s")
    send_cmd(ard, cmd)
    time.sleep(duration_s)
    stop_motors(ard)
    time.sleep(SETTLE_TIME_S)


# ===========================================================
# LiDAR parsing and scan classification
# ===========================================================

# Current navigation code assumed: theta 0 = front, negative = left, positive = right.
# These sectors cover the five movement-related directions TaskManager cares about.
SCAN_SECTORS = [
    {"direction": "left", "center_deg": -90.0, "min_deg": -112.5, "max_deg": -67.5},
    {"direction": "front_left", "center_deg": -45.0, "min_deg": -67.5, "max_deg": -22.5},
    {"direction": "front", "center_deg": 0.0, "min_deg": -22.5, "max_deg": 22.5},
    {"direction": "front_right", "center_deg": 45.0, "min_deg": 22.5, "max_deg": 67.5},
    {"direction": "right", "center_deg": 90.0, "min_deg": 67.5, "max_deg": 112.5},
]


def parse_lidar_line(line: str) -> Optional[Tuple[float, float]]:
    """Return (theta_deg, distance_m) from an ultra_simple output line."""
    line = line.strip()
    if not line.startswith("theta"):
        return None
    try:
        parts = line.replace(":", " ").split()
        theta = float(parts[1])
        dist_mm = float(parts[3])
        if dist_mm <= 0:
            return None
        return theta, dist_mm / 1000.0
    except Exception as exc:
        log_nav_error("lidar_parse_error", exc, {"line": line})
        return None


def normalize_lidar_theta(theta_deg: float) -> float:
    # Apply installation offset, then convert to [-180,+180].
    return (theta_deg + LIDAR_YAW_OFFSET_DEG + 180.0) % 360.0 - 180.0


def sector_for_theta(theta_norm: float) -> Optional[Dict[str, float]]:
    for sector in SCAN_SECTORS:
        if sector["min_deg"] <= theta_norm <= sector["max_deg"]:
            return sector
    return None


def read_adjacent_scan(lidar_proc: subprocess.Popen, robot_lat: float, robot_lon: float, heading_deg: Optional[float]) -> List[Dict[str, Any]]:
    """
    Read a short LiDAR window and summarize occupancy in left/front-left/front/front-right/right.

    Navigation owns the threshold and GPS projection. For occupied sectors, it reports
    approximate object_lat/object_lon so TaskManager can convert that point into a grid cell.
    Clear sectors represent adjacent 1.0 m grid cells; TaskManager infers their cells
    from robot pose and direction.
    """
    nearest: Dict[str, Optional[float]] = {s["direction"]: None for s in SCAN_SECTORS}
    end = time.time() + LIDAR_WINDOW_SECONDS

    stdout = lidar_proc.stdout
    if stdout is None:
        return build_scan_payload(nearest, robot_lat, robot_lon, heading_deg)

    while time.time() < end:
        try:
            ready, _, _ = select.select([stdout], [], [], 0.02)
        except Exception as exc:
            log_nav_error("lidar_select_error", exc)
            break
        if not ready:
            continue

        line = stdout.readline()
        if not line:
            continue
        parsed = parse_lidar_line(line)
        if parsed is None:
            continue

        theta, dist_m = parsed
        if dist_m < LIDAR_MIN_VALID_M or dist_m > LIDAR_MAX_REPORT_M:
            continue
        theta_norm = normalize_lidar_theta(theta)
        sector = sector_for_theta(theta_norm)
        if sector is None:
            continue

        name = str(sector["direction"])
        if nearest[name] is None or dist_m < float(nearest[name]):
            nearest[name] = dist_m

    return build_scan_payload(nearest, robot_lat, robot_lon, heading_deg)


def build_scan_payload(
    nearest: Dict[str, Optional[float]],
    robot_lat: float,
    robot_lon: float,
    heading_deg: Optional[float],
) -> List[Dict[str, Any]]:
    scan: List[Dict[str, Any]] = []
    for sector in SCAN_SECTORS:
        direction = str(sector["direction"])
        rel = float(sector["center_deg"])
        dist_m = nearest.get(direction)
        occupied = dist_m is not None and dist_m <= LIDAR_OCCUPIED_THRESHOLD_M

        object_lat = None
        object_lon = None
        if occupied and heading_deg is not None and dist_m is not None:
            # Occupied object GPS uses the measured LiDAR distance, not CELL_SIZE_M.
            obj_bearing = (float(heading_deg) + rel) % 360.0
            object_lat, object_lon = project_gps(robot_lat, robot_lon, obj_bearing, dist_m)

        scan.append(
            {
                "direction": direction,
                "relative_heading_deg": rel,
                "occupied": bool(occupied),
                "object_lat": object_lat,
                "object_lon": object_lon,
                "distance_m": dist_m if occupied else None,
            }
        )
    return scan


def intended_sector_for_target(current_heading: Optional[float], target_bearing: float) -> str:
    """Return which scan sector the next movement is roughly aimed through."""
    if current_heading is None:
        return "front"
    err = normalize_angle_deg(target_bearing - current_heading)
    if err < -67.5:
        return "left"
    if err < -22.5:
        return "front_left"
    if err <= 22.5:
        return "front"
    if err <= 67.5:
        return "front_right"
    return "right"


def sector_item(scan: List[Dict[str, Any]], direction: str) -> Optional[Dict[str, Any]]:
    for item in scan:
        if item.get("direction") == direction:
            return item
    return None


# ===========================================================
# MQTT navigation node
# ===========================================================

class NavigationNode:
    def __init__(self):
        self.state = SharedState()
        self.client = mqtt.Client(client_id=f"navigation_{ROBOT_ID}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

    def on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            print(f"{ts()} MQTT connection failed rc={rc}")
            log_nav_error("mqtt_connection_failed", f"rc={rc}", {"rc": rc})
            return
        client.subscribe(TOPIC_CMD_WAYPOINT)
        client.subscribe(TOPIC_CMD_STOP)
        log_nav_event("mqtt_connected", {"topics": [TOPIC_CMD_WAYPOINT, TOPIC_CMD_STOP]})
        print(f"{ts()} MQTT connected. Subscribed to {TOPIC_CMD_WAYPOINT} and {TOPIC_CMD_STOP}")

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except Exception as exc:
            print(f"{ts()} Bad JSON on {msg.topic}: {exc}")
            log_nav_error("mqtt_bad_json", exc, {"topic": msg.topic, "payload": msg.payload.decode("utf-8", errors="ignore")})
            return

        if msg.topic == TOPIC_CMD_STOP:
            print(f"{ts()} STOP command received: {data}")
            log_nav_event("stop_command_received", {"payload": data})
            self.state.request_stop()
            return

        if msg.topic == TOPIC_CMD_WAYPOINT:
            try:
                wp_data = data["waypoint"]
                wp = Waypoint(
                    lat=float(wp_data["lat"]),
                    lon=float(wp_data["lon"]),
                    cell_x=wp_data.get("cell_x"),
                    cell_y=wp_data.get("cell_y"),
                    timestamp=float(data.get("timestamp", time.time())),
                )
                self.state.set_waypoint(wp)
                log_nav_event(
                    "waypoint_received",
                    {
                        "waypoint_lat": wp.lat,
                        "waypoint_lon": wp.lon,
                        "waypoint_cell_x": wp.cell_x,
                        "waypoint_cell_y": wp.cell_y,
                        "payload": data,
                    },
                )
                print(
                    f"{ts()} New waypoint: lat={wp.lat:.7f}, lon={wp.lon:.7f}, "
                    f"cell=({wp.cell_x},{wp.cell_y})"
                )
            except Exception as exc:
                print(f"{ts()} Invalid waypoint payload: {exc}; data={data}")
                log_nav_error("waypoint_received_invalid", exc, {"payload": data})

    def connect(self):
        self.client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, MQTT_KEEPALIVE)
        self.client.loop_start()

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()

    def publish_status(self, fix: Fix, heading_deg: Optional[float], status: str, extra: Optional[Dict[str, Any]] = None):
        payload: Dict[str, Any] = {
            "robot_id": ROBOT_ID,
            "lat": fix.lat,
            "lon": fix.lon,
            "heading_deg": heading_deg,
            "status": status,
            "timestamp": time.time(),
        }
        if extra:
            payload.update(extra)
        self.client.publish(TOPIC_NAV_STATUS, json.dumps(payload))
        log_nav_event("nav_status_published", {"status": status, "payload": payload})

    def publish_startup(self, fix: Fix, heading_deg: Optional[float]):
        payload = {
            "robot_id": ROBOT_ID,
            "lat": fix.lat,
            "lon": fix.lon,
            "heading_deg": heading_deg,
            "timestamp": time.time(),
        }
        self.client.publish(TOPIC_NAV_STARTUP, json.dumps(payload))
        log_nav_event("startup_published", {"payload": payload})
        print(f"{ts()} Published startup GPS lat={fix.lat:.7f}, lon={fix.lon:.7f}, heading={heading_deg}")

    def publish_adjacent_scan(self, fix: Fix, heading_deg: Optional[float], scan: List[Dict[str, Any]]):
        active_wp, _ = self.state.snapshot()
        payload = {
            "robot_id": ROBOT_ID,
            "robot_lat": fix.lat,
            "robot_lon": fix.lon,
            "heading_deg": heading_deg,
            "scan": scan,
            "timestamp": time.time(),
        }
        self.client.publish(TOPIC_NAV_ADJACENT_SCAN, json.dumps(payload))
        log_nav_event(
            "lidar_scan_summary",
            {
                "robot_lat": fix.lat,
                "robot_lon": fix.lon,
                "robot_heading_deg": heading_deg,
                "robot_cell_x": active_wp.cell_x if active_wp else None,
                "robot_cell_y": active_wp.cell_y if active_wp else None,
                "scan": scan,
                "payload": payload,
            },
        )
        for item in scan:
            if item.get("occupied"):
                log_nav_event("lidar_occupied_detection", item)
            else:
                log_nav_event("lidar_free_detection", item)


# ===========================================================
# Main execution
# ===========================================================

def run():
    nav = NavigationNode()
    running = True
    shutdown_reason = "normal_exit"
    log_nav_event("program_start", nav_program_start_fields())

    def handle_signal(signum, frame):
        nonlocal running, shutdown_reason
        print(f"{ts()} Signal {signum} received; stopping navigation.")
        running = False
        shutdown_reason = f"signal_{signum}"
        nav.state.request_stop()

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    gps = open_serial_with_retry(GPS_PORT, GPS_BAUD, "GPS")
    ard = open_serial_with_retry(ARD_PORT, ARD_BAUD, "Arduino")

    print(f"{ts()} Starting LiDAR process: {' '.join(LIDAR_CMD)}")
    lidar_proc = subprocess.Popen(
        LIDAR_CMD,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        universal_newlines=True,
        bufsize=1,
    )
    print(f"{ts()} LiDAR process started")

    print(f"{ts()} Waiting {ARDUINO_BOOT_WAIT_S:.1f}s for Arduino boot/reset after serial open")
    time.sleep(ARDUINO_BOOT_WAIT_S)
    stop_motors(ard)
    time.sleep(ARDUINO_POST_STOP_WAIT_S)

    nav.connect()

    startup_published = False
    last_status_publish = 0.0
    last_quality_debug = 0.0
    navigation_was_usable = False
    rtk_loss_reported = False
    last_valid_heading_deg: Optional[float] = None
    heading_acquisition_failed = False

    try:
        print(f"{ts()} UGV_Navigation.py started for ROBOT_ID={ROBOT_ID}")
        while running:
            wp, stop_requested = nav.state.snapshot()

            if stop_requested:
                stop_motors(ard)
                time.sleep(IDLE_SLEEP_S)
                continue

            fix = read_fix(gps, timeout_s=1.2)
            if fix is None:
                stop_motors(ard)
                now = time.time()
                if navigation_was_usable:
                    print(f"{ts()} RTK FIX LOST")
                    gps_quality_alarm()
                    rtk_loss_reported = True
                navigation_was_usable = False
                if now - last_quality_debug >= STATUS_PUBLISH_INTERVAL_S:
                    print(f"{ts()} NO UBX NAV-PVT FIX -> S")
                    # Do not publish /nav/status without usable lat/lon/heading.
                    last_quality_debug = now
                time.sleep(NO_FIX_SLEEP_S)
                continue

            usable_fix = fix_is_navigation_usable(fix)
            print_fix_debug(fix, "FIXED" if usable_fix else "NOT_USABLE")

            if not usable_fix:
                stop_motors(ard)
                if navigation_was_usable:
                    print(f"{ts()} RTK FIX LOST")
                    gps_quality_alarm()
                    rtk_loss_reported = True
                navigation_was_usable = False
                time.sleep(NO_FIX_SLEEP_S)
                continue

            if rtk_loss_reported:
                print(f"{ts()} RTK FIX RESTORED")
                rtk_loss_reported = False
            navigation_was_usable = True

            if heading_acquisition_failed and not startup_published:
                stop_motors(ard)
                now = time.time()
                if now - last_status_publish >= STATUS_PUBLISH_INTERVAL_S:
                    nav.publish_status(fix, None, "heading_acquisition_failed")
                    last_status_publish = now
                time.sleep(IDLE_SLEEP_S)
                continue

            if INITIAL_HEADING_ENABLED and not startup_published and last_valid_heading_deg is None:
                result = acquire_initial_heading(
                    gps,
                    ard,
                    nav,
                    lambda: running and not nav.state.snapshot()[1],
                )
                if result is None:
                    heading_acquisition_failed = True
                    time.sleep(IDLE_SLEEP_S)
                    continue

                fix, acquired_heading = result
                last_valid_heading_deg = acquired_heading

            current_heading, last_valid_heading_deg, heading_updated = filtered_heading_from_fix(
                fix,
                last_valid_heading_deg,
            )
            if heading_updated:
                print(
                    f"{ts()} Heading updated from NAV-PVT headMot: "
                    f"speed={fix.ground_speed_mps:.2f} heading={current_heading:.1f}"
                )
            else:
                held_heading = "None" if current_heading is None else f"{current_heading:.1f}"
                print(
                    f"{ts()} Speed below heading threshold: "
                    f"speed={fix.ground_speed_mps:.2f} holding heading={held_heading}"
                )

            if current_heading is None:
                stop_motors(ard)
                print(
                    f"{ts()} No valid heading yet; waiting for speed above "
                    f"MIN_HEADING_SPEED_MPS before navigating"
                )
                time.sleep(IDLE_SLEEP_S)
                continue

            if not startup_published:
                # Do not move before a valid GPS startup message has been published.
                nav.publish_startup(fix, current_heading)
                startup_published = True

            if wp is None:
                stop_motors(ard)
                now = time.time()
                if now - last_status_publish >= STATUS_PUBLISH_INTERVAL_S:
                    nav.publish_status(fix, current_heading, "idle")
                    last_status_publish = now
                time.sleep(IDLE_SLEEP_S)
                continue

            dist_m = haversine_m(fix.lat, fix.lon, wp.lat, wp.lon)
            target_bearing = bearing_deg(fix.lat, fix.lon, wp.lat, wp.lon)
            log_nav_event(
                "waypoint_active",
                {
                    "waypoint_lat": wp.lat,
                    "waypoint_lon": wp.lon,
                    "waypoint_cell_x": wp.cell_x,
                    "waypoint_cell_y": wp.cell_y,
                    "current_lat": fix.lat,
                    "current_lon": fix.lon,
                    "current_heading_deg": current_heading,
                    "distance_to_waypoint_m": dist_m,
                },
            )

            # Adjacent LiDAR scans are only published at waypoint stops for this phase.
            # The environment is assumed static.
            if dist_m <= WAYPOINT_RADIUS_M:
                stop_motors(ard)
                print(
                    f"{ts()} WAYPOINT REACHED dist={dist_m:.2f}m "
                    f"cell=({wp.cell_x},{wp.cell_y}) -> S"
                )
                log_nav_event(
                    "waypoint_reached",
                    {
                        "waypoint_lat": wp.lat,
                        "waypoint_lon": wp.lon,
                        "waypoint_cell_x": wp.cell_x,
                        "waypoint_cell_y": wp.cell_y,
                        "current_lat": fix.lat,
                        "current_lon": fix.lon,
                        "current_heading_deg": current_heading,
                        "distance_to_waypoint_m": dist_m,
                    },
                )
                scan = read_adjacent_scan(lidar_proc, fix.lat, fix.lon, current_heading)
                nav.publish_adjacent_scan(fix, current_heading, scan)
                nav.publish_status(
                    fix,
                    current_heading,
                    "waypoint_reached",
                    extra={
                        "waypoint": {
                            "lat": wp.lat,
                            "lon": wp.lon,
                            "cell_x": wp.cell_x,
                            "cell_y": wp.cell_y,
                        }
                    },
                )
                nav.state.clear_waypoint()
                time.sleep(IDLE_SLEEP_S)
                continue

            # Use adjacent scan for emergency occupancy detection only while driving.
            scan = read_adjacent_scan(lidar_proc, fix.lat, fix.lon, current_heading)

            intended_sector = intended_sector_for_target(current_heading, target_bearing)
            intended_item = sector_item(scan, intended_sector)
            intended_occupied = bool(intended_item and intended_item.get("occupied"))
            intended_distance = intended_item.get("distance_m") if intended_item else None

            if intended_distance is not None and intended_distance <= LIDAR_HARD_STOP_M:
                stop_motors(ard)
                log_nav_event(
                    "blocked_or_hard_stop",
                    {
                        "reason": "hard_stop",
                        "intended_sector": intended_sector,
                        "intended_distance": intended_distance,
                        "current_lat": fix.lat,
                        "current_lon": fix.lon,
                        "current_heading_deg": current_heading,
                        "active_waypoint_cell_x": wp.cell_x,
                        "active_waypoint_cell_y": wp.cell_y,
                    },
                )
                nav.publish_status(
                    fix,
                    current_heading,
                    "blocked",
                    extra={"reason": "hard_stop", "intended_sector": intended_sector},
                )
                print(f"{ts()} HARD STOP obstacle in {intended_sector} at {intended_distance:.2f}m")
                nav.state.clear_waypoint()
                time.sleep(IDLE_SLEEP_S)
                continue

            if intended_occupied:
                stop_motors(ard)
                log_nav_event(
                    "blocked_or_hard_stop",
                    {
                        "reason": "intended_cell_occupied",
                        "intended_sector": intended_sector,
                        "intended_distance": intended_distance,
                        "current_lat": fix.lat,
                        "current_lon": fix.lon,
                        "current_heading_deg": current_heading,
                        "active_waypoint_cell_x": wp.cell_x,
                        "active_waypoint_cell_y": wp.cell_y,
                    },
                )
                nav.publish_status(
                    fix,
                    current_heading,
                    "blocked",
                    extra={"reason": "intended_cell_occupied", "intended_sector": intended_sector},
                )
                print(
                    f"{ts()} Intended movement sector occupied: {intended_sector}; "
                    f"dist={intended_distance} -> S, report blocked"
                )
                nav.state.clear_waypoint()
                time.sleep(IDLE_SLEEP_S)
                continue

            # Publish moving status periodically.
            now = time.time()
            if now - last_status_publish >= STATUS_PUBLISH_INTERVAL_S:
                nav.publish_status(
                    fix,
                    current_heading,
                    "moving",
                    extra={
                        "distance_to_waypoint_m": dist_m,
                        "target_bearing_deg": target_bearing,
                        "intended_sector": intended_sector,
                    },
                )
                last_status_publish = now

            print(
                f"{ts()} NAV dist={dist_m:.2f}m target={target_bearing:.1f}deg "
                f"heading={current_heading} intended={intended_sector}"
            )

            # Burst control copied conceptually from current navigation:
            # use UBX NAV-PVT headMot to decide whether to steer left/right/forward.

            err = normalize_angle_deg(target_bearing - current_heading)
            if err > BIG_ERR_DEG:
                print(f"{ts()} Large right error {err:.1f}deg -> R burst")
                do_burst(ard, "R", TURN_BURST_S)
            elif err < -BIG_ERR_DEG:
                print(f"{ts()} Large left error {err:.1f}deg -> L burst")
                do_burst(ard, "L", TURN_BURST_S)
            elif err > SMALL_ERR_DEG:
                print(f"{ts()} Small right error {err:.1f}deg -> R burst")
                do_burst(ard, "R", SMALL_TURN_BURST_S)
            elif err < -SMALL_ERR_DEG:
                print(f"{ts()} Small left error {err:.1f}deg -> L burst")
                do_burst(ard, "L", SMALL_TURN_BURST_S)
            else:
                print(f"{ts()} Aligned err={err:.1f}deg -> F burst")
                do_burst(ard, "F", F_BURST_S)

    finally:
        print(f"{ts()} Shutting down navigation")
        log_nav_event("program_shutdown", nav_program_shutdown_fields(shutdown_reason))
        stop_motors(ard)
        try:
            nav.disconnect()
        except Exception:
            pass
        try:
            lidar_proc.terminate()
            lidar_proc.wait(timeout=2.0)
        except Exception:
            try:
                lidar_proc.kill()
            except Exception:
                pass
        try:
            gps.close()
        except Exception:
            pass
        try:
            ard.close()
        except Exception:
            pass


if __name__ == "__main__":
    run()

