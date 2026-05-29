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
import select
import signal
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

GPS_PORT = "/dev/ttyTHS1"
GPS_BAUD = 38400

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
WAYPOINT_RADIUS_M = 0.10
GPS_MIN_MOVE_FOR_HEADING_M = 0.45
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
    raw: str = ""
    timestamp: float = 0.0


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


def parse_lat_lon(lat_str: str, lat_hemi: str, lon_str: str, lon_hemi: str) -> Tuple[Optional[float], Optional[float]]:
    try:
        if not lat_str or not lon_str:
            return None, None
        lat_deg = int(lat_str[:2])
        lat_min = float(lat_str[2:])
        lon_deg = int(lon_str[:3])
        lon_min = float(lon_str[3:])
        lat = lat_deg + lat_min / 60.0
        lon = lon_deg + lon_min / 60.0
        if lat_hemi == "S":
            lat = -lat
        if lon_hemi == "W":
            lon = -lon
        return lat, lon
    except Exception:
        return None, None


def open_serial_with_retry(port: str, baud: int, name: str) -> serial.Serial:
    while True:
        try:
            ser = serial.Serial(port, baud, timeout=1)
            print(f"{ts()} Opened {name} on {port} @ {baud}")
            return ser
        except Exception as exc:
            print(f"{ts()} Failed opening {name} on {port}: {exc}")
            time.sleep(1.0)


def send_cmd(ser: serial.Serial, cmd: str):
    ser.write(cmd.encode("ascii"))
    ser.flush()


def stop_motors(ard: serial.Serial):
    try:
        send_cmd(ard, "S")
    except Exception as exc:
        print(f"{ts()} Failed to send stop command: {exc}")


def read_fix(gps: serial.Serial, timeout_s: float = 1.5) -> Optional[Fix]:
    end = time.time() + timeout_s
    while time.time() < end:
        try:
            line = gps.readline().decode("ascii", errors="ignore").strip()
        except Exception:
            continue
        if line.startswith("$GNRMC") or line.startswith("$GPRMC"):
            parts = line.split(",")
            if len(parts) >= 7:
                status = parts[2]
                lat, lon = parse_lat_lon(parts[3], parts[4], parts[5], parts[6])
                if status == "A" and lat is not None and lon is not None:
                    return Fix(lat=lat, lon=lon, raw=line, timestamp=time.time())
    return None


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
    except Exception:
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
        except Exception:
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
            return
        client.subscribe(TOPIC_CMD_WAYPOINT)
        client.subscribe(TOPIC_CMD_STOP)
        print(f"{ts()} MQTT connected. Subscribed to {TOPIC_CMD_WAYPOINT} and {TOPIC_CMD_STOP}")

    def on_message(self, client, userdata, msg):
        try:
            data = json.loads(msg.payload.decode("utf-8"))
        except Exception as exc:
            print(f"{ts()} Bad JSON on {msg.topic}: {exc}")
            return

        if msg.topic == TOPIC_CMD_STOP:
            print(f"{ts()} STOP command received: {data}")
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
                print(
                    f"{ts()} New waypoint: lat={wp.lat:.7f}, lon={wp.lon:.7f}, "
                    f"cell=({wp.cell_x},{wp.cell_y})"
                )
            except Exception as exc:
                print(f"{ts()} Invalid waypoint payload: {exc}; data={data}")

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

    def publish_startup(self, fix: Fix, heading_deg: Optional[float]):
        payload = {
            "robot_id": ROBOT_ID,
            "lat": fix.lat,
            "lon": fix.lon,
            "heading_deg": heading_deg,
            "timestamp": time.time(),
        }
        self.client.publish(TOPIC_NAV_STARTUP, json.dumps(payload))
        print(f"{ts()} Published startup GPS lat={fix.lat:.7f}, lon={fix.lon:.7f}, heading={heading_deg}")

    def publish_adjacent_scan(self, fix: Fix, heading_deg: Optional[float], scan: List[Dict[str, Any]]):
        payload = {
            "robot_id": ROBOT_ID,
            "robot_lat": fix.lat,
            "robot_lon": fix.lon,
            "heading_deg": heading_deg,
            "scan": scan,
            "timestamp": time.time(),
        }
        self.client.publish(TOPIC_NAV_ADJACENT_SCAN, json.dumps(payload))


def estimate_motion_heading(prev_fix: Optional[Fix], current_fix: Fix, previous_heading: Optional[float]) -> Optional[float]:
    if prev_fix is None:
        return previous_heading
    moved_m = haversine_m(prev_fix.lat, prev_fix.lon, current_fix.lat, current_fix.lon)
    if moved_m < GPS_MIN_MOVE_FOR_HEADING_M:
        return previous_heading
    return bearing_deg(prev_fix.lat, prev_fix.lon, current_fix.lat, current_fix.lon)


# ===========================================================
# Main execution
# ===========================================================

def run():
    nav = NavigationNode()
    running = True

    def handle_signal(signum, frame):
        nonlocal running
        print(f"{ts()} Signal {signum} received; stopping navigation.")
        running = False
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

    prev_fix: Optional[Fix] = None
    current_heading: Optional[float] = None
    startup_published = False
    last_status_publish = 0.0

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
                if now - last_status_publish >= STATUS_PUBLISH_INTERVAL_S:
                    print(f"{ts()} NO GPS FIX -> S")
                    # Cannot publish /nav/status without lat/lon fields because TaskManager expects them.
                    last_status_publish = now
                time.sleep(NO_FIX_SLEEP_S)
                continue

            current_heading = estimate_motion_heading(prev_fix, fix, current_heading)

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
                prev_fix = fix
                time.sleep(IDLE_SLEEP_S)
                continue

            dist_m = haversine_m(fix.lat, fix.lon, wp.lat, wp.lon)
            target_bearing = bearing_deg(fix.lat, fix.lon, wp.lat, wp.lon)

            # Adjacent LiDAR scans are only published at waypoint stops for this phase.
            # The environment is assumed static.
            if dist_m <= WAYPOINT_RADIUS_M:
                stop_motors(ard)
                print(
                    f"{ts()} WAYPOINT REACHED dist={dist_m:.2f}m "
                    f"cell=({wp.cell_x},{wp.cell_y}) -> S"
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
                prev_fix = fix
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
                nav.publish_status(
                    fix,
                    current_heading,
                    "blocked",
                    extra={"reason": "hard_stop", "intended_sector": intended_sector},
                )
                print(f"{ts()} HARD STOP obstacle in {intended_sector} at {intended_distance:.2f}m")
                nav.state.clear_waypoint()
                prev_fix = fix
                time.sleep(IDLE_SLEEP_S)
                continue

            if intended_occupied:
                stop_motors(ard)
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
                prev_fix = fix
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
            # use GPS motion heading to decide whether to steer left/right/forward.
            if current_heading is None:
                print(f"{ts()} No reliable motion heading yet -> initial F burst")
                do_burst(ard, "F", F_BURST_S)
                prev_fix = fix
                continue

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

            prev_fix = fix

    finally:
        print(f"{ts()} Shutting down navigation")
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
