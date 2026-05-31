#!/usr/bin/env python3
"""
TaskManager.py

UGV task manager derived from the Auction-Greedy testbed algorithm.

What was preserved from Auction-Greedy:
- unsearched / observed / obstacle-style grid states
- target and clue probability maps
- probability decay around detections using the same 1 / (1 + d)**EXP form
- auction/greedy goal selection
- peer position / intent / goal / clue / target sharing concepts
- A* planning around searched cells, non-traversable cells, and temporary peer obstacles

This script is intended to run on the Jetson Nano. The navigation process is
responsible for GPS, LiDAR, Arduino motor commands, and physical movement.
This task manager only assigns GPS waypoints and maintains the search/belief map.

TODO:
4. Confirm LiDAR yaw offset so 0 degrees actually means vehicle-forward.
6. Test TaskManager + Navigation MQTT message flow with mosquitto_sub.
7. Try to implement a PID controller instead of burst commands
9. Implement camera script publishing only numeric interrogation result.
10. Test single-robot full loop: startup GPS -> map init -> waypoint -> scan -> reached -> next waypoint.
11. Later: implement ESP-NOW bridge for peer/outgoing and peer/incoming.
12. Later: switch SINGLE_ROBOT_MODE to False and test two-robot startup/map sharing.
- need sweep width because not only coordinates robot passes through are searched. We now have degrees of search.
- Maybe only turn cameras on when LiDAR detects something.

"""

import json
import math
import os
import time
import heapq
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Set, Any

try:
    import paho.mqtt.client as mqtt
except ImportError as exc:
    raise SystemExit(
        "Missing dependency: paho-mqtt. Install with: pip3 install paho-mqtt"
    ) from exc


# ===========================================================
# USER-EDITABLE CONFIGURATION
# ===========================================================

ROBOT_ID = "00"                 # USER EDIT: this UGV's ID
SINGLE_ROBOT_MODE = True        # USER EDIT: True for solo testing, False for two-car operation

# Only used when SINGLE_ROBOT_MODE == True
SINGLE_ROBOT_MAP_WIDTH_M = 20.0
SINGLE_ROBOT_MAP_HEIGHT_M = 20.0

MULTI_ROBOT_PEER_IDS = ["01"]   # USER EDIT: other UGV IDs when SINGLE_ROBOT_MODE is False
PEER_IDS = [] if SINGLE_ROBOT_MODE else MULTI_ROBOT_PEER_IDS
TEAM_IDS = [ROBOT_ID] + PEER_IDS

MQTT_BROKER_HOST = "localhost"  # local broker on this Jetson
MQTT_BROKER_PORT = 1883
MQTT_KEEPALIVE = 10

TRIAL_ID = time.strftime("%Y%m%d-%H%M%S")
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(SCRIPT_DIR, "logs")
TASK_EVENT_LOG_FILE = "task-events.jsonl"
TASK_ERROR_LOG_FILE = "task-errors.jsonl"

CELL_SIZE_M = 1.0               # USER EDIT: 1.0 m x 1.0 m grid cells
# Grid resolution is 1.0 m per cell. TaskManager and Navigation must use the same value.

# Auction-Greedy probability / planning tunables preserved from the original code.
REWARD_FACTOR = 5.0
TARGET_DECAY_EXP = 1.0          # same target correlation rate as Auction-Greedy
CLUE_DECAY_EXP = 0.0            # same placeholder as Auction-Greedy; edit later
CLUE_POD = 1.0                  # placeholder probability of detecting a clue if present
CLUE_VALUE_WEIGHT = 0.0         # kept from original CLUE_VALUE_WEIGHT behavior
VISITED_STEP_PENALTY = 4.0
UNKNOWN_OBJECT_BONUS = 10.0     # USER EDIT: priority for camera interrogation targets
INTERROGATION_TIMEOUT_S = 5.0   # USER EDIT: wait time for camera result after observation arrival

# Heading-aware grid planning tunables for the RC-car-style UGV.
# The planner still operates on grid cells, but each A* state also carries one
# of eight compass headings. Steering is constrained to forward-left, forward,
# or forward-right so the path cannot pivot in place or instantly reverse.
TURN_COST_STRAIGHT = 0.0
TURN_COST_45_DEG = 0.4
TURN_COST_90_DEG = 3.0
TURN_COST_135_DEG = 50.0
TURN_COST_180_DEG = 1e6
MAX_STEERING_DELTA_PER_STEP = 1

# USER-EDITABLE CLUE CLASSES
# The object detection side can send just a numeric clue_id.
# For now, clue_id == 1 is treated as target found, per your instruction.
CLUE_CLASSES = {
    1: "target_found",          # USER EDIT: currently object/clue ID 1 stops the mission
    2: "placeholder_clue_2",    # USER EDIT
    3: "placeholder_clue_3",    # USER EDIT
    4: "placeholder_clue_4",    # USER EDIT
}
TARGET_FOUND_CLUE_IDS = {1}

# USER-EDITABLE LIDAR BLOCKING RULE
# Navigation owns LiDAR thresholding and occupied-cell geometry.
# TaskManager only updates the persistent planning map.
BLOCKED_CELLS_PERMANENT = True  # permanent until next trial / restart

# Runtime behavior
MAIN_LOOP_SLEEP_S = 0.1
PLAN_INTERVAL_S = 1.0
STATUS_TIMEOUT_S = 5.0
STARTUP_TIMEOUT_S = None        # None = wait indefinitely for my GPS and peer startup GPS


# ===========================================================
# USER-EDITABLE MESSAGE FORMAT: MQTT TOPICS
# ===========================================================
# These are formats I created for this TaskManager. Edit these names if your
# navigation, detection, or ESP-NOW bridge uses different topic names.

TOPIC_NAV_STARTUP = f"/ugv/{ROBOT_ID}/nav/startup"       # nav -> task_manager
TOPIC_NAV_STARTUP_ACK = f"/ugv/{ROBOT_ID}/nav/startup_ack"  # task_manager -> nav
TOPIC_NAV_STATUS = f"/ugv/{ROBOT_ID}/nav/status"         # nav -> task_manager
TOPIC_NAV_ADJACENT_SCAN = f"/ugv/{ROBOT_ID}/nav/adjacent_scan"  # nav -> task_manager
TOPIC_CLUE = f"/ugv/{ROBOT_ID}/detections/clue"          # detector -> task_manager
TOPIC_INTERROGATION = f"/ugv/{ROBOT_ID}/detections/interrogation"  # detector -> task_manager

TOPIC_CMD_WAYPOINT = f"/ugv/{ROBOT_ID}/cmd/waypoint"     # task_manager -> nav
TOPIC_CMD_STOP = f"/ugv/{ROBOT_ID}/cmd/stop"             # task_manager -> nav

# Peer bridge topics. The ESP32/ESP-NOW bridge can subscribe to peer/outgoing
# and transmit the JSON payload over ESP-NOW. It can publish received peer JSON
# messages to peer/incoming.
TOPIC_PEER_OUT = f"/ugv/{ROBOT_ID}/peer/outgoing"        # task_manager -> ESP-NOW bridge
TOPIC_PEER_IN = f"/ugv/{ROBOT_ID}/peer/incoming"         # ESP-NOW bridge -> task_manager


# ===========================================================
# Persistent JSONL logging
# ===========================================================

def trial_log_filename(base_name: str) -> str:
    root, ext = os.path.splitext(base_name)
    filename = f"{root}-{TRIAL_ID}{ext or '.jsonl'}"
    return os.path.join(LOG_DIR, filename)


TASK_EVENT_LOG_PATH = trial_log_filename(TASK_EVENT_LOG_FILE)
TASK_ERROR_LOG_PATH = trial_log_filename(TASK_ERROR_LOG_FILE)


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple, set)):
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
            print(f"WARNING failed writing log {path}: {exc}")
        except Exception:
            pass


def log_task_event(event_type: str, fields: Optional[Dict[str, Any]] = None):
    write_jsonl(
        TASK_EVENT_LOG_PATH,
        {
            "timestamp": time.time(),
            "trial_id": TRIAL_ID,
            "robot_id": ROBOT_ID,
            "event_type": event_type,
            "fields": fields or {},
        },
    )


def log_task_error(event_type: str, exc_or_message, fields: Optional[Dict[str, Any]] = None):
    write_jsonl(
        TASK_ERROR_LOG_PATH,
        {
            "timestamp": time.time(),
            "trial_id": TRIAL_ID,
            "robot_id": ROBOT_ID,
            "event_type": event_type,
            "error": repr(exc_or_message),
            "fields": fields or {},
        },
    )


def task_program_start_fields() -> Dict[str, Any]:
    return {
        "trial_id": TRIAL_ID,
        "robot_id": ROBOT_ID,
        "script_name": os.path.basename(__file__),
        "timestamp": time.time(),
        "script_dir": SCRIPT_DIR,
        "log_dir": LOG_DIR,
        "current_working_directory": os.getcwd(),
        "event_log": TASK_EVENT_LOG_PATH,
        "error_log": TASK_ERROR_LOG_PATH,
        "key_config": {
            "log_dir": LOG_DIR,
            "single_robot_mode": SINGLE_ROBOT_MODE,
            "single_robot_map_width_m": SINGLE_ROBOT_MAP_WIDTH_M,
            "single_robot_map_height_m": SINGLE_ROBOT_MAP_HEIGHT_M,
            "multi_robot_peer_ids": MULTI_ROBOT_PEER_IDS,
            "mqtt_broker_host": MQTT_BROKER_HOST,
            "mqtt_broker_port": MQTT_BROKER_PORT,
            "cell_size_m": CELL_SIZE_M,
            "reward_factor": REWARD_FACTOR,
            "target_decay_exp": TARGET_DECAY_EXP,
            "clue_decay_exp": CLUE_DECAY_EXP,
            "visited_step_penalty": VISITED_STEP_PENALTY,
            "unknown_object_bonus": UNKNOWN_OBJECT_BONUS,
            "interrogation_timeout_s": INTERROGATION_TIMEOUT_S,
            "plan_interval_s": PLAN_INTERVAL_S,
        },
    }


def task_program_shutdown_fields(reason: str) -> Dict[str, Any]:
    return {
        "trial_id": TRIAL_ID,
        "robot_id": ROBOT_ID,
        "script_name": os.path.basename(__file__),
        "timestamp": time.time(),
        "reason": reason,
    }


# ===========================================================
# Type aliases and grid states
# ===========================================================

Cell = Tuple[int, int]
GPS = Tuple[float, float]

# Cell semantics now separate observation/search state from traversability.
# The old testbed used "searched" to mean "the robot physically occupied this
# cell." For the UGV, LiDAR/camera can search cells before the vehicle enters
# them, so planning and probability updates must treat observed cells directly.
CELL_UNSEARCHED = 0          # not observed yet; may be traversable after nav verifies it
CELL_FREE_SEARCHED = 1       # observed empty/free and traversable
CELL_OCCUPIED_UNKNOWN = 2    # LiDAR object, camera not classified; non-traversable, high priority
CELL_OCCUPIED_CLUE = 3       # non-traversable object that contains a clue/feature
CELL_OBSTACLE = 4            # camera-confirmed non-clue hard obstacle; non-traversable
CELL_TARGET_FOUND = 5        # optional marker; mission stop logic still owns target handling

# Heading representation:
# index 0..7 represents N, NE, E, SE, S, SW, W, NW respectively.
# Positive y is north in the relative grid, positive x is east.
DIRS8 = (
    (0, 1),    # N
    (1, 1),    # NE
    (1, 0),    # E
    (1, -1),   # SE
    (0, -1),   # S
    (-1, -1),  # SW
    (-1, 0),   # W
    (-1, 1),   # NW
)
STEERING_DELTAS = (-MAX_STEERING_DELTA_PER_STEP, 0, MAX_STEERING_DELTA_PER_STEP)


@dataclass
class Pose:
    lat: float
    lon: float
    heading_deg: Optional[float] = None
    timestamp: float = 0.0


@dataclass
class PeerState:
    startup_gps: Optional[GPS] = None
    pos_gps: Optional[GPS] = None
    pos_cell: Optional[Cell] = None
    intent_cell: Optional[Cell] = None
    goal_cell: Optional[Cell] = None
    last_seen: float = 0.0


@dataclass
class GoalSelection:
    movement_cell: Cell
    interrogation_target: Optional[Cell] = None
    required_goal_heading: Optional[int] = None


class RelativeGeoGrid:
    """
    Deterministic GPS-aligned relative map.

    Two startup GPS coordinates are treated as opposite corners. The origin is
    chosen deterministically as the southwest-ish min-lat/min-lon corner, so both
    robots construct the same grid even if they start from different corners.

    Axes:
    - x = east/west meters
    - y = north/south meters
    - each cell = CELL_SIZE_M x CELL_SIZE_M

    Map boundaries correspond exactly to the defined search area. No boundary
    padding is applied.
    """

    def __init__(self, gps_a: GPS, gps_b: GPS, cell_size_m: float):
        lat1, lon1 = gps_a
        lat2, lon2 = gps_b

        self.min_lat = min(lat1, lat2)
        self.max_lat = max(lat1, lat2)
        self.min_lon = min(lon1, lon2)
        self.max_lon = max(lon1, lon2)
        self.origin_lat = self.min_lat
        self.origin_lon = self.min_lon
        self.cell_size_m = cell_size_m

        width_m = max(0.01, haversine_m(self.origin_lat, self.min_lon, self.origin_lat, self.max_lon))
        height_m = max(0.01, haversine_m(self.min_lat, self.origin_lon, self.max_lat, self.origin_lon))

        self.width_cells = max(1, math.ceil(width_m / cell_size_m))
        self.height_cells = max(1, math.ceil(height_m / cell_size_m))
        self.n_cells = self.width_cells * self.height_cells

    def in_bounds(self, cell: Cell) -> bool:
        x, y = cell
        return 0 <= x < self.width_cells and 0 <= y < self.height_cells

    def idx(self, cell: Cell) -> int:
        x, y = cell
        if not self.in_bounds(cell):
            raise IndexError(f"cell out of bounds: {cell}")
        return y * self.width_cells + x

    def cell_from_idx(self, i: int) -> Cell:
        return (i % self.width_cells, i // self.width_cells)

    def gps_to_cell(self, lat: float, lon: float) -> Optional[Cell]:
        if lat < self.min_lat or lat > self.max_lat or lon < self.min_lon or lon > self.max_lon:
            return None
        east_m = signed_east_m(self.origin_lat, self.origin_lon, lon)
        north_m = signed_north_m(self.origin_lat, self.origin_lon, lat)
        x = int(math.floor(east_m / self.cell_size_m))
        y = int(math.floor(north_m / self.cell_size_m))
        # Points exactly on the north/east search boundary belong to the last
        # in-area cell. This preserves exact map bounds without adding padding.
        x = min(max(x, 0), self.width_cells - 1)
        y = min(max(y, 0), self.height_cells - 1)
        cell = (x, y)
        return cell if self.in_bounds(cell) else None

    def cell_to_gps(self, cell: Cell) -> GPS:
        x, y = cell
        # Use cell center. The 0.5 factor is a unitless center-of-cell offset,
        # not a grid-resolution assumption.
        east_m = (x + 0.5) * self.cell_size_m
        north_m = (y + 0.5) * self.cell_size_m
        lat = self.origin_lat + meters_to_lat_delta(north_m)
        lon = self.origin_lon + meters_to_lon_delta(east_m, self.origin_lat)
        return (lat, lon)

    def clamp_cell(self, cell: Cell) -> Cell:
        x, y = cell
        return (
            min(max(x, 0), self.width_cells - 1),
            min(max(y, 0), self.height_cells - 1),
        )


class AuctionGreedyTaskManager:
    def __init__(self):
        self.client = mqtt.Client(client_id=f"task_manager_{ROBOT_ID}")
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.map: Optional[RelativeGeoGrid] = None
        self.grid: List[int] = []
        self.target_p: List[float] = []
        self.clue_p: List[float] = []
        self.prob_map: List[float] = []

        self.my_startup_gps: Optional[GPS] = None
        self.my_pose: Optional[Pose] = None
        self.my_cell: Optional[Cell] = None
        self.heading_index: Optional[int] = None

        self.peers: Dict[str, PeerState] = {pid: PeerState() for pid in PEER_IDS}
        self.current_goal: Optional[Cell] = None
        self.current_goal_cell: Optional[Cell] = None
        self.current_interrogation_target: Optional[Cell] = None
        self.current_required_goal_heading: Optional[int] = None
        self.interrogation_wait_started_at: Optional[float] = None
        self.current_path: List[Cell] = []
        self.last_waypoint_cell: Optional[Cell] = None

        self.clue_cells: List[Cell] = []
        self.first_clue_seen = False
        self.found_target = False
        self.target_location_gps: Optional[GPS] = None

        self.last_plan_time = 0.0
        self.mission_started_time: Optional[float] = None

        # Metrics placeholders retained from the original algorithm's spirit.
        self.unique_cells_count = 0
        self.system_visits: Dict[Cell, int] = {}
        self.path_replan_count = 0
        self.goal_replan_count = 0
        self.yield_count = 0
        self.clue_count = 0
        self.obstacle_count = 0

    def clear_current_goal(self):
        self.current_goal = None
        self.current_goal_cell = None
        self.current_interrogation_target = None
        self.current_required_goal_heading = None
        self.interrogation_wait_started_at = None

    # =======================================================
    # MQTT setup and callbacks
    # =======================================================

    def start(self):
        shutdown_reason = "normal_exit"
        log_task_event("program_start", task_program_start_fields())
        self.client.connect(MQTT_BROKER_HOST, MQTT_BROKER_PORT, MQTT_KEEPALIVE)
        self.client.loop_start()
        print(f"TaskManager {ROBOT_ID} connected to MQTT {MQTT_BROKER_HOST}:{MQTT_BROKER_PORT}")
        if SINGLE_ROBOT_MODE:
            print("MODE: SINGLE ROBOT")
            print(f"Map size: {SINGLE_ROBOT_MAP_WIDTH_M:.1f} m x {SINGLE_ROBOT_MAP_HEIGHT_M:.1f} m")
            print("Waiting only for local GPS startup.")
        else:
            print("MODE: MULTI ROBOT")
            print("Waiting for local GPS and peer startup GPS.")

        try:
            while not self.found_target:
                now = time.time()
                if self.map is not None and self.my_cell is not None:
                    if now - self.last_plan_time >= PLAN_INTERVAL_S:
                        self.check_interrogation_timeout(now)
                        if self.interrogation_wait_started_at is None:
                            self.plan_and_publish_if_needed()
                        self.last_plan_time = now
                time.sleep(MAIN_LOOP_SLEEP_S)
        finally:
            if self.found_target:
                shutdown_reason = "target_found"
            self.publish_stop(reason="task_manager_exit")
            log_task_event("program_shutdown", task_program_shutdown_fields(shutdown_reason))
            self.client.loop_stop()
            self.client.disconnect()

    def on_connect(self, client, userdata, flags, rc):
        if rc != 0:
            print(f"MQTT connection failed rc={rc}")
            log_task_error("mqtt_connection_failed", f"rc={rc}", {"rc": rc})
            return

        # USER-EDITABLE COMMUNICATION PROCESSING:
        # Subscriptions I created for navigation, clue detection, and peer bridge.
        topics = [
            TOPIC_NAV_STARTUP,
            TOPIC_NAV_STATUS,
            TOPIC_NAV_ADJACENT_SCAN,
            TOPIC_CLUE,
            TOPIC_INTERROGATION,
        ]
        if not SINGLE_ROBOT_MODE:
            topics.append(TOPIC_PEER_IN)
        for topic in topics:
            client.subscribe(topic)
            print(f"Subscribed: {topic}")
        log_task_event("mqtt_connected", {"topics": topics})

    def on_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode("utf-8"))
        except Exception as exc:
            print(f"Bad JSON on {msg.topic}: {exc}")
            log_task_error("mqtt_bad_json", exc, {"topic": msg.topic, "payload": msg.payload.decode("utf-8", errors="ignore")})
            return

        try:
            if msg.topic == TOPIC_NAV_STARTUP:
                self.handle_nav_startup(payload)
            elif msg.topic == TOPIC_NAV_STATUS:
                self.handle_nav_status(payload)
            elif msg.topic == TOPIC_NAV_ADJACENT_SCAN:
                self.handle_nav_adjacent_scan(payload)
            elif msg.topic == TOPIC_CLUE:
                self.handle_clue_detection(payload)
            elif msg.topic == TOPIC_INTERROGATION:
                self.handle_interrogation_result(payload)
            elif msg.topic == TOPIC_PEER_IN:
                self.handle_peer_message(payload)
        except Exception as exc:
            print(f"Error handling {msg.topic}: {exc}")
            log_task_error("mqtt_handler_error", exc, {"topic": msg.topic, "payload": payload})

    # =======================================================
    # USER-EDITABLE MESSAGE FORMAT: navigation input
    # =======================================================

    def handle_nav_startup(self, data: Dict[str, Any]):
        """
        Navigation -> TaskManager startup message.

        USER-EDITABLE FORMAT I CREATED:
        Topic: /ugv/<ROBOT_ID>/nav/startup
        {
            "robot_id": "00",
            "lat": 32.0,
            "lon": -117.0,
            "heading_deg": 90.0,
            "timestamp": 123456.7
        }
        """
        lat = float(data["lat"])
        lon = float(data["lon"])
        self.my_startup_gps = (lat, lon)
        self.my_pose = Pose(lat, lon, data.get("heading_deg"), data.get("timestamp", time.time()))
        self.heading_index = heading_index_from_degrees(data.get("heading_deg"))
        log_task_event("nav_startup_received", {"payload": data, "startup_gps": self.my_startup_gps, "heading_index": self.heading_index})
        self.publish_nav_startup_ack(data)

        self.publish_peer("startup", {
            "lat": lat,
            "lon": lon,
            "heading_deg": data.get("heading_deg"),
        })
        self.try_initialize_map()

    def publish_nav_startup_ack(self, startup_payload: Dict[str, Any]):
        payload = {
            "robot_id": ROBOT_ID,
            "status": "startup_received",
            "startup_timestamp": startup_payload.get("timestamp"),
            "timestamp": time.time(),
        }
        self.client.publish(TOPIC_NAV_STARTUP_ACK, json.dumps(payload))
        log_task_event("nav_startup_ack_published", {"payload": payload})

    def handle_nav_status(self, data: Dict[str, Any]):
        """
        Navigation -> TaskManager status message.

        USER-EDITABLE FORMAT I CREATED:
        Topic: /ugv/<ROBOT_ID>/nav/status
        {
            "robot_id": "00",
            "lat": 32.0,
            "lon": -117.0,
            "heading_deg": 90.0,
            "status": "startup|moving|waypoint_reached|blocked|failed|no_fix",
            "timestamp": 123456.7
        }

        Keep status simple. Observation-based search normally arrives through
        /ugv/<ROBOT_ID>/nav/adjacent_scan. Optional observed_free_cells and
        occupied_unknown_cells are still accepted for compatibility, but the
        adjacent_scan topic is the preferred local LiDAR observation format.
        """
        lat = float(data["lat"])
        lon = float(data["lon"])
        heading_deg = data.get("heading_deg")
        status = data.get("status", "unknown")
        self.my_pose = Pose(lat, lon, heading_deg, data.get("timestamp", time.time()))
        log_task_event("nav_status_received", {"payload": data})

        if self.map is None:
            return

        new_cell = self.map.gps_to_cell(lat, lon)
        if new_cell is None:
            return

        # Navigation owns heading estimation. Treat heading_deg as authoritative
        # ZED-F9R NAV-PVT headMot and do not derive heading from GPS motion.
        measured_heading = heading_index_from_degrees(heading_deg)
        if measured_heading is not None:
            self.heading_index = measured_heading

        self.my_cell = new_cell
        self.publish_peer("position", {"lat": lat, "lon": lon, "heading_deg": heading_deg})
        self.process_observed_cells(data)

        if status == "waypoint_reached":
            if self.mark_free_searched(new_cell):
                log_task_event("cell_marked_free_searched", {"robot_cell": new_cell, "observed_cell": new_cell, "source": "navigation", "robot_heading_deg": heading_deg})
                self.publish_cell_update(new_cell, "free_searched", source="navigation")
            if self.last_waypoint_cell == new_cell:
                self.last_waypoint_cell = None
            log_task_event("waypoint_reached_processed", {"robot_cell": new_cell, "heading_deg": heading_deg, "payload": data})
            if self.current_interrogation_target is not None:
                # Interrogation arrival: the reached waypoint is free/searched,
                # but the occupied target is not searched-empty. The final
                # approach heading was planned to face the target, so TaskManager
                # waits for perception rather than publishing another waypoint.
                self.interrogation_wait_started_at = time.time()
                log_task_event("interrogation_started", {"observation_cell": new_cell, "target_cell": self.current_interrogation_target})
                print(
                    "Waiting for interrogation result for "
                    f"target={self.current_interrogation_target} from observation_cell={new_cell}"
                )
                return
            self.plan_and_publish_if_needed(force=True)
        elif status in {"failed", "blocked"}:
            self.clear_current_goal()
            self.current_path = []
            self.path_replan_count += 1
            log_task_event("replan_triggered", {"reason": status, "robot_cell": new_cell})
            self.plan_and_publish_if_needed(force=True)

    def process_observed_cells(self, data: Dict[str, Any]):
        # Optional nav/perception fields let LiDAR/camera search cells around
        # the vehicle before it occupies them. This replaces the old assumption
        # that "searched" only means "robot drove into that exact cell."
        for cell in self.cells_from_observation_list(data.get("observed_free_cells", [])):
            if self.mark_free_searched(cell):
                log_task_event("cell_marked_free_searched", {"observed_cell": cell, "source": "lidar_status_compat"})
                self.publish_cell_update(cell, "free_searched", source="lidar")
        for cell in self.cells_from_observation_list(data.get("occupied_unknown_cells", [])):
            if self.mark_occupied_unknown(cell):
                log_task_event("cell_marked_occupied_unknown", {"observed_cell": cell, "source": "lidar_status_compat"})
                self.publish_cell_update(cell, "occupied_unknown", source="lidar")

    def cells_from_observation_list(self, observations: Any) -> List[Cell]:
        if self.map is None or not isinstance(observations, list):
            return []
        cells = []
        for item in observations:
            cell = self.cell_from_observation(item)
            if cell is not None:
                cells.append(cell)
        return cells

    def cell_from_observation(self, item: Any) -> Optional[Cell]:
        assert self.map is not None
        if not isinstance(item, dict):
            return None
        if "cell_x" in item and "cell_y" in item:
            cell = (int(item["cell_x"]), int(item["cell_y"]))
            return cell if self.map.in_bounds(cell) else None
        if "lat" in item and "lon" in item:
            return self.map.gps_to_cell(float(item["lat"]), float(item["lon"]))
        return None

    def check_interrogation_timeout(self, now: Optional[float] = None):
        if self.interrogation_wait_started_at is None:
            return
        if self.current_interrogation_target is None:
            self.interrogation_wait_started_at = None
            return
        now = time.time() if now is None else now
        if now - self.interrogation_wait_started_at < INTERROGATION_TIMEOUT_S:
            return

        target = self.current_interrogation_target
        print(f"Interrogation timed out for target={target}; marking obstacle and replanning.")
        log_task_event("interrogation_timeout", {"target_cell": target})
        self.mark_obstacle(target)
        log_task_event("cell_marked_obstacle", {"observed_cell": target, "source": "camera", "reason": "interrogation_timeout"})
        self.publish_cell_update(target, "obstacle", source="camera", reason="interrogation_timeout")
        self.clear_current_goal()
        self.current_path = []
        self.path_replan_count += 1
        self.plan_and_publish_if_needed(force=True)

    def handle_nav_adjacent_scan(self, data: Dict[str, Any]):
        """
        Navigation -> TaskManager adjacent-cell LiDAR scan result.

        USER-EDITABLE FORMAT I CREATED:
        Topic: /ugv/<ROBOT_ID>/nav/adjacent_scan
        {
            "robot_id": "00",
            "robot_lat": 32.0,
            "robot_lon": -117.0,
            "heading_deg": 90.0,
            "scan": [
                {
                    "direction": "front_left",
                    "relative_heading_deg": -45,
                    "occupied": true,
                    "object_lat": 32.0,
                    "object_lon": -117.0,
                    "distance_m": 0.42
                },
                {
                    "direction": "front",
                    "relative_heading_deg": 0,
                    "occupied": false,
                    "object_lat": null,
                    "object_lon": null,
                    "distance_m": null
                },
                {
                    "direction": "front_right",
                    "relative_heading_deg": 45,
                    "occupied": false,
                    "object_lat": null,
                    "object_lon": null,
                    "distance_m": null
                }
            ],
            "timestamp": 123456.7
        }

        Navigation owns LiDAR thresholding and sensor geometry. For occupied
        detections, navigation should provide object_lat/object_lon; TaskManager
        does not infer occupied object location from direction in this handler.
        Direction is used only for clear adjacent cells. TaskManager
        trusts these adjacent scan results and only updates grid/belief/planning
        state. Occupied cells are non-traversable pending camera classification;
        free adjacent cells are marked observed empty/free without requiring the
        robot to physically occupy them. Every cell state change is shared with
        peers through an explicit cell_update message.
        """
        if self.map is None:
            return

        robot_lat = float(data["robot_lat"])
        robot_lon = float(data["robot_lon"])
        heading_deg = data.get("heading_deg")
        robot_cell = self.map.gps_to_cell(robot_lat, robot_lon)
        if robot_cell is None:
            return
        log_task_event(
            "adjacent_scan_received",
            {
                "robot_lat": robot_lat,
                "robot_lon": robot_lon,
                "robot_cell": robot_cell,
                "heading_deg": heading_deg,
                "scan": data.get("scan", []),
            },
        )

        self.my_pose = Pose(robot_lat, robot_lon, heading_deg, data.get("timestamp", time.time()))
        measured_heading = heading_index_from_degrees(heading_deg)
        if measured_heading is not None:
            self.heading_index = measured_heading
        self.my_cell = robot_cell

        changed = False
        for item in data.get("scan", []):
            if not isinstance(item, dict):
                continue

            occupied = bool(item.get("occupied", False))
            if occupied:
                cell = self.occupied_scan_cell(item)
                log_task_event(
                    "adjacent_scan_item_processed",
                    {
                        "robot_cell": robot_cell,
                        "direction": item.get("direction"),
                        "occupied": True,
                        "object_lat": item.get("object_lat"),
                        "object_lon": item.get("object_lon"),
                        "object_cell": cell,
                        "distance_m": item.get("distance_m"),
                    },
                )
                if cell is None:
                    continue
                if self.mark_occupied_unknown(cell):
                    log_task_event(
                        "cell_marked_occupied_unknown",
                        {
                            "robot_cell": robot_cell,
                            "observed_cell": cell,
                            "source": "lidar",
                            "direction": item.get("direction"),
                            "distance_m": item.get("distance_m"),
                            "robot_heading_deg": heading_deg,
                        },
                    )
                    self.publish_cell_update(cell, "occupied_unknown", source="lidar")
                    changed = True
                continue

            cell = self.free_adjacent_scan_cell(robot_cell, heading_deg, item)
            log_task_event(
                "adjacent_scan_item_processed",
                {
                    "robot_cell": robot_cell,
                    "direction": item.get("direction"),
                    "occupied": False,
                    "free_adjacent_cell": cell,
                    "distance_m": item.get("distance_m"),
                },
            )
            if cell is None:
                continue
            before = self.grid[self.map.idx(cell)]
            if self.mark_free_searched(cell):
                log_task_event(
                    "cell_marked_free_searched",
                    {
                        "robot_cell": robot_cell,
                        "observed_cell": cell,
                        "source": "lidar",
                        "direction": item.get("direction"),
                        "robot_heading_deg": heading_deg,
                    },
                )
                self.publish_cell_update(cell, "free_searched", source="lidar")
            if self.grid[self.map.idx(cell)] != before:
                changed = True

        if changed:
            self.current_path = []
            self.path_replan_count += 1
            log_task_event("replan_triggered", {"reason": "adjacent_scan_changed_map", "robot_cell": robot_cell})

        # Adjacent scans are treated as waypoint-stop observations. Planning the
        # next waypoint is normally triggered by waypoint_reached, not by the
        # scan itself.
        if self.last_waypoint_cell is not None:
            return

        self.plan_and_publish_if_needed(force=True)

    def occupied_scan_cell(self, item: Dict[str, Any]) -> Optional[Cell]:
        assert self.map is not None
        if item.get("object_lat") is None or item.get("object_lon") is None:
            return None
        return self.map.gps_to_cell(float(item["object_lat"]), float(item["object_lon"]))

    def free_adjacent_scan_cell(self, robot_cell: Cell, heading_deg: Optional[float], item: Dict[str, Any]) -> Optional[Cell]:
        assert self.map is not None
        relative_heading = item.get("relative_heading_deg")
        direction = item.get("direction")
        heading_idx = scan_heading_index(heading_deg, direction, relative_heading)
        if heading_idx is None:
            return None
        dx, dy = DIRS8[heading_idx]
        cell = (robot_cell[0] + dx, robot_cell[1] + dy)
        return cell if self.map.in_bounds(cell) else None

    # =======================================================
    # USER-EDITABLE MESSAGE FORMAT: clue detection input
    # =======================================================

    def handle_clue_detection(self, data: Dict[str, Any]):
        """
        Object/face/clue detector -> TaskManager message.

        USER-EDITABLE FORMAT I CREATED:
        Topic: /ugv/<ROBOT_ID>/detections/clue
        {
            "robot_id": "00",
            "clue_id": 2,             # numeric placeholder class ID from detector
            "confidence": 0.85,
            "lat": 32.0,
            "lon": -117.0,
            "timestamp": 123456.7
        }

        For now no distinction is made between object and face detection.
        clue_id == 1 is assumed to mean target found.
        """
        if self.map is None:
            return

        clue_id = int(data.get("clue_id", 0))
        confidence = float(data.get("confidence", 1.0))
        lat = float(data["lat"])
        lon = float(data["lon"])
        cell = self.map.gps_to_cell(lat, lon)
        if cell is None:
            return
        log_task_event("clue_detection_received", {"payload": data, "cell": cell})

        if clue_id in TARGET_FOUND_CLUE_IDS:
            self.grid[self.map.idx(cell)] = CELL_TARGET_FOUND
            self.handle_target_found(lat, lon, source="local_clue_detection", clue_id=clue_id)
        else:
            self.mark_occupied_clue(cell)
            log_task_event("cell_marked_occupied_clue", {"observed_cell": cell, "source": "camera", "clue_id": clue_id, "confidence": confidence})
            self.publish_cell_update(
                cell,
                "occupied_clue",
                source="camera",
                clue_id=clue_id,
                confidence=confidence,
            )

        self.clue_count += 1
        if cell not in self.clue_cells:
            self.clue_cells.append(cell)
        self.first_clue_seen = True
        self.clue_probability_field(cell, confidence=confidence)
        self.update_prob_map()

        self.plan_and_publish_if_needed(force=True)

    def handle_interrogation_result(self, data: Any):
        """
        Camera/feature interrogation result for the currently faced occupied cell.

        Simple camera message format:
            0  -> no recognized clue / unknown object, treat as hard obstacle
            1  -> target found
            2+ -> clue / feature ID

        The camera does NOT send GPS, cell coordinates, timestamp, result string, or confidence.
        TaskManager already knows which cell is being interrogated through
        self.current_interrogation_target.
        """
        if self.map is None:
            return

        if self.current_interrogation_target is None:
            print(f"Ignoring interrogation result with no active interrogation target: {data}")
            return

        cell = self.current_interrogation_target
        log_task_event("interrogation_result_received", {"raw_result": data, "target_cell": cell})

        try:
            clue_id = int(data)
        except (TypeError, ValueError):
            print(f"Ignoring invalid interrogation result: {data}")
            return

        lat, lon = self.map.cell_to_gps(cell)

        # 0 means the camera did not identify a useful clue/target.
        # For this phase, treat that occupied cell as a confirmed hard obstacle.
        if clue_id == 0:
            self.mark_obstacle(cell)
            log_task_event("cell_marked_obstacle", {"observed_cell": cell, "source": "camera", "reason": "no_recognized_clue"})
            self.publish_cell_update(cell, "obstacle", source="camera", reason="no_recognized_clue")

        # clue_id 1 means target found.
        elif clue_id in TARGET_FOUND_CLUE_IDS:
            self.grid[self.map.idx(cell)] = CELL_TARGET_FOUND
            self.interrogation_wait_started_at = None
            self.handle_target_found(lat, lon, source="local_interrogation", clue_id=clue_id)
            return

        # Any other positive ID is treated as a clue / semantic feature.
        elif clue_id > 0:
            self.mark_occupied_clue(cell)
            log_task_event("cell_marked_occupied_clue", {"observed_cell": cell, "source": "camera", "clue_id": clue_id, "confidence": 1.0})
            self.clue_count += 1

            if cell not in self.clue_cells:
                self.clue_cells.append(cell)

            self.first_clue_seen = True
            self.clue_probability_field(cell, confidence=1.0)
            self.update_prob_map()

            self.publish_cell_update(
                cell,
                "occupied_clue",
                source="camera",
                clue_id=clue_id,
                confidence=1.0,
            )

        else:
            print(f"Ignoring invalid negative clue_id={clue_id}")
            return

        self.clear_current_goal()
        self.current_path = []
        self.path_replan_count += 1
        log_task_event("replan_triggered", {"reason": "interrogation_result", "target_cell": cell})
        self.plan_and_publish_if_needed(force=True)

    # =======================================================
    # USER-EDITABLE MESSAGE FORMAT: peer messages
    # =======================================================

    def publish_peer(self, msg_type: str, fields: Dict[str, Any]):
        """
        TaskManager -> ESP-NOW bridge outgoing message.

        USER-EDITABLE FORMAT I CREATED:
        Topic: /ugv/<ROBOT_ID>/peer/outgoing
        {
            "sender": "00",
            "type": "startup|position|intent|goal|cell_update|target",
            ... type-specific fields ...
        }

        cell_update payload:
        {
            "sender": "00",
            "type": "cell_update",
            "cell_state": "free_searched|occupied_unknown|occupied_clue|obstacle",
            "lat": 32.0,
            "lon": -117.0,
            "cell_x": 10,
            "cell_y": 12,
            "source": "lidar|camera|peer|navigation",
            "clue_id": null,
            "confidence": null,
            "timestamp": 123456.7
        }

        Peer searched/occupied/clue/obstacle map knowledge is shared through explicit
        cell_update messages. Peer position is not proof that a cell was
        searched.
        """
        if SINGLE_ROBOT_MODE:
            return
        payload = {
            "sender": ROBOT_ID,
            "type": msg_type,
            "timestamp": time.time(),
            **fields,
        }
        self.client.publish(TOPIC_PEER_OUT, json.dumps(payload))

    def publish_cell_update(
        self,
        cell: Cell,
        cell_state: str,
        source: str,
        clue_id: Optional[int] = None,
        confidence: Optional[float] = None,
        reason: Optional[str] = None,
    ):
        if self.map is None:
            return
        lat, lon = self.map.cell_to_gps(cell)
        payload = {
            "cell_state": cell_state,
            "lat": lat,
            "lon": lon,
            "cell_x": cell[0],
            "cell_y": cell[1],
            "source": source,
            "clue_id": clue_id,
            "confidence": confidence,
        }
        if reason is not None:
            payload["reason"] = reason
        self.publish_peer("cell_update", payload)

    def handle_peer_message(self, data: Dict[str, Any]):
        """
        ESP-NOW bridge -> TaskManager incoming peer message.

        USER-EDITABLE COMMUNICATION PROCESSING:
        This replaces Auction-Greedy's compact UART topic parser. It preserves
        the same coordination concepts: position, intent, goal, explicit
        cell_update map observations, and target alert.
        """
        if SINGLE_ROBOT_MODE:
            return
        sender = data.get("sender")
        if sender == ROBOT_ID or sender not in self.peers:
            return
        msg_type = data.get("type")
        peer = self.peers[sender]
        peer.last_seen = time.time()

        if msg_type == "startup":
            peer.startup_gps = (float(data["lat"]), float(data["lon"]))
            self.try_initialize_map()

        elif msg_type == "position":
            if self.map is None:
                return
            lat, lon = float(data["lat"]), float(data["lon"])
            new_cell = self.map.gps_to_cell(lat, lon)
            peer.pos_gps = (lat, lon)
            peer.pos_cell = new_cell
            # Peer position is no longer treated as proof that a cell was
            # searched. Observation results are shared through explicit
            # cell_update messages. Peer position is only a temporary collision
            # avoidance obstacle.

        elif msg_type == "intent":
            if self.map is None:
                return
            # Peer intent is only a temporary collision avoidance obstacle.
            peer.intent_cell = self.map.gps_to_cell(float(data["lat"]), float(data["lon"]))

        elif msg_type == "goal":
            if self.map is None:
                return
            # Peer goal is only for task allocation conflict avoidance.
            peer.goal_cell = self.map.gps_to_cell(float(data["lat"]), float(data["lon"]))
            if self.current_goal is not None and peer.goal_cell == self.current_goal:
                self.clear_current_goal()
                self.goal_replan_count += 1

        elif msg_type == "clue":
            if self.map is None:
                return
            clue_id = int(data.get("clue_id", 0))
            lat, lon = float(data["lat"]), float(data["lon"])
            if clue_id in TARGET_FOUND_CLUE_IDS:
                self.handle_target_found(lat, lon, source="peer_clue_detection", clue_id=clue_id)
                return
            # Non-target peer clue map updates now arrive through explicit
            # cell_update messages so peer observations carry cell state.

        elif msg_type == "blocked":
            if self.map is None:
                return
            # Legacy peer blocked messages are not used as map truth. Occupied,
            # occupied, free, and clue observations are shared through
            # explicit cell_update messages.

        elif msg_type == "cell_update":
            if self.map is None:
                return
            cell = None
            # Prefer GPS conversion over raw peer cell coordinates. Both robots
            # should build the same grid, but GPS is the canonical shared frame.
            if "lat" in data and "lon" in data:
                cell = self.map.gps_to_cell(float(data["lat"]), float(data["lon"]))
            if cell is None and "cell_x" in data and "cell_y" in data:
                candidate = (int(data["cell_x"]), int(data["cell_y"]))
                if self.map.in_bounds(candidate):
                    cell = candidate
            if cell is None:
                return

            cell_state = str(data.get("cell_state", "")).strip().lower()
            changed = False
            if cell_state == "free_searched":
                changed = self.mark_free_searched(cell)
            elif cell_state == "occupied_unknown":
                changed = self.mark_occupied_unknown(cell)
            elif cell_state == "occupied_clue":
                changed = self.mark_occupied_clue(cell)
                clue_id = data.get("clue_id")
                confidence = float(data.get("confidence", 1.0) or 1.0)
                if cell not in self.clue_cells:
                    self.clue_cells.append(cell)
                self.first_clue_seen = True
                self.clue_probability_field(cell, confidence=confidence)
                self.update_prob_map()
            elif cell_state == "obstacle":
                self.mark_obstacle(cell)
                changed = True
            else:
                return
            if changed:
                self.clear_current_goal()
                self.current_path = []
                self.path_replan_count += 1

        elif msg_type == "target":
            self.handle_target_found(float(data["lat"]), float(data["lon"]), source="peer_target", clue_id=None)

    # =======================================================
    # Map initialization
    # =======================================================

    def try_initialize_map(self):
        if self.map is not None or self.my_startup_gps is None:
            return

        if SINGLE_ROBOT_MODE:
            east_corner = project_gps(
                self.my_startup_gps[0],
                self.my_startup_gps[1],
                90.0,
                SINGLE_ROBOT_MAP_WIDTH_M,
            )
            opposite = project_gps(
                east_corner[0],
                east_corner[1],
                0.0,
                SINGLE_ROBOT_MAP_HEIGHT_M,
            )
            self.map = RelativeGeoGrid(self.my_startup_gps, opposite, CELL_SIZE_M)
            self.allocate_maps()

            self.my_cell = self.map.gps_to_cell(*self.my_startup_gps)
            if self.my_cell is not None:
                self.record_searched_cell(self.my_cell)

            self.mission_started_time = time.time()
            print(
                f"Initialized single-robot map: {self.map.width_cells} x {self.map.height_cells} cells "
                f"({CELL_SIZE_M} m/cell), origin lat/lon=({self.map.origin_lat:.7f}, {self.map.origin_lon:.7f})"
            )
            log_task_event(
                "map_initialized",
                {
                    "mode": "single",
                    "width_cells": self.map.width_cells,
                    "height_cells": self.map.height_cells,
                    "origin_lat": self.map.origin_lat,
                    "origin_lon": self.map.origin_lon,
                    "my_cell": self.my_cell,
                },
            )
            self.plan_and_publish_if_needed(force=True)
            return

        peer_startups = [p.startup_gps for p in self.peers.values() if p.startup_gps is not None]
        if not peer_startups:
            return

        # For now assume one peer and opposite corners. If more peers exist later,
        # use the farthest startup point from this robot as the opposite corner.
        opposite = max(peer_startups, key=lambda gps: haversine_m(self.my_startup_gps[0], self.my_startup_gps[1], gps[0], gps[1]))
        self.map = RelativeGeoGrid(self.my_startup_gps, opposite, CELL_SIZE_M)
        self.allocate_maps()

        self.my_cell = self.map.gps_to_cell(*self.my_startup_gps)
        if self.my_cell is not None:
            self.record_searched_cell(self.my_cell)
            self.update_target_on_miss(self.my_cell)

        for pid, peer in self.peers.items():
            if peer.startup_gps is not None:
                peer.pos_gps = peer.startup_gps
                peer.pos_cell = self.map.gps_to_cell(*peer.startup_gps)

        self.mission_started_time = time.time()
        print(
            f"Initialized relative map: {self.map.width_cells} x {self.map.height_cells} cells "
            f"({CELL_SIZE_M} m/cell), origin lat/lon=({self.map.origin_lat:.7f}, {self.map.origin_lon:.7f})"
        )
        log_task_event(
            "map_initialized",
            {
                "mode": "multi",
                "width_cells": self.map.width_cells,
                "height_cells": self.map.height_cells,
                "origin_lat": self.map.origin_lat,
                "origin_lon": self.map.origin_lon,
                "my_cell": self.my_cell,
                "peer_cells": {pid: peer.pos_cell for pid, peer in self.peers.items()},
            },
        )
        self.plan_and_publish_if_needed(force=True)

    def allocate_maps(self):
        assert self.map is not None
        n = self.map.n_cells
        self.grid = [CELL_UNSEARCHED] * n
        self.target_p = [1.0 / n] * n
        self.clue_p = [1.0 / n] * n
        self.prob_map = [1.0 / n] * n

    # =======================================================
    # Preserved Auction-Greedy map / probability logic
    # =======================================================

    def renorm(self, arr: List[float]):
        total = sum(arr)
        if total <= 0.0:
            val = 1.0 / len(arr)
            for i in range(len(arr)):
                arr[i] = val
            return
        inv = 1.0 / total
        for i in range(len(arr)):
            arr[i] *= inv

    def recompute_value_map(self):
        for i in range(len(self.prob_map)):
            self.prob_map[i] = self.target_p[i] + (self.clue_p[i] * CLUE_POD * CLUE_VALUE_WEIGHT)
        self.renorm(self.prob_map)

    def update_prob_map(self):
        assert self.map is not None
        if self.clue_cells:
            for y in range(self.map.height_cells):
                for x in range(self.map.width_cells):
                    cell = (x, y)
                    i = self.map.idx(cell)
                    if is_observed_empty(self.grid[i]):
                        self.target_p[i] = 0.0
                        continue
                    s = 0.0
                    for clue_cell in self.clue_cells:
                        d = manhattan_cell(cell, clue_cell)
                        s += 1.0 / ((1.0 + d) ** TARGET_DECAY_EXP)
                    self.target_p[i] = s
            self.renorm(self.target_p)
        self.recompute_value_map()

    def update_clue_on_miss(self, cell: Cell):
        if self.map is None or not self.map.in_bounds(cell):
            return
        i = self.map.idx(cell)
        p_i = self.clue_p[i]
        if p_i <= 0.0:
            return
        self.clue_p[i] = p_i * (1.0 - CLUE_POD)
        self.renorm(self.clue_p)
        self.recompute_value_map()

    def update_target_on_miss(self, cell: Cell):
        if self.map is None or not self.map.in_bounds(cell):
            return
        i = self.map.idx(cell)
        if self.target_p[i] <= 0.0:
            return
        self.target_p[i] = 0.0
        self.renorm(self.target_p)
        self.recompute_value_map()

    def clue_probability_field(self, clue_cell: Cell, confidence: float = 1.0):
        """
        Preserved from Auction-Greedy's clue_probability_field(). The decay is
        the same 1 / (1 + Manhattan distance)**CLUE_DECAY_EXP form and naturally
        ends at the grid bounds.
        """
        assert self.map is not None
        for y in range(self.map.height_cells):
            for x in range(self.map.width_cells):
                cell = (x, y)
                i = self.map.idx(cell)
                d = manhattan_cell(cell, clue_cell)
                bump = confidence * (1.0 / ((1.0 + d) ** CLUE_DECAY_EXP))
                self.clue_p[i] += bump
        self.clue_p[self.map.idx(clue_cell)] = 0.0
        self.renorm(self.clue_p)
        self.recompute_value_map()

    def record_searched_cell(self, cell: Cell):
        # Compatibility wrapper for older call sites. "Searched" now means
        # observed empty/free, not necessarily newly occupied by this robot.
        return self.mark_free_searched(cell)

    def mark_free_searched(self, cell: Cell) -> bool:
        if self.map is None or not self.map.in_bounds(cell):
            return False
        i = self.map.idx(cell)
        if not is_traversable_cell_state(self.grid[i]):
            return False
        already_free = self.grid[i] == CELL_FREE_SEARCHED
        first_visit = cell not in self.system_visits
        self.system_visits[cell] = self.system_visits.get(cell, 0) + 1
        if first_visit:
            self.unique_cells_count += 1
        self.grid[i] = CELL_FREE_SEARCHED
        # Probability misses now attach to observed free cells, not just cells
        # physically occupied by the robot.
        self.update_target_on_miss(cell)
        self.update_clue_on_miss(cell)
        return not already_free

    def mark_occupied_unknown(self, cell: Cell) -> bool:
        if self.map is None or not self.map.in_bounds(cell):
            return False
        i = self.map.idx(cell)
        if self.grid[i] in {CELL_OCCUPIED_CLUE, CELL_OBSTACLE, CELL_TARGET_FOUND}:
            return False
        if self.grid[i] == CELL_OCCUPIED_UNKNOWN:
            return False
        self.grid[i] = CELL_OCCUPIED_UNKNOWN
        # LiDAR has observed an unclassified object here, so the empty-cell miss
        # update is not appropriate. Keep target/clue probability for later
        # camera interrogation, but make the cell non-traversable for A*.
        self.recompute_value_map()
        print(f"Marked occupied unknown cell: {cell}")
        return True

    def mark_occupied_clue(self, cell: Cell) -> bool:
        if self.map is None or not self.map.in_bounds(cell):
            return False
        i = self.map.idx(cell)
        if self.grid[i] == CELL_OCCUPIED_CLUE:
            return False
        self.grid[i] = CELL_OCCUPIED_CLUE
        # A clue-bearing occupied cell is searched/observed evidence, but it is
        # not a drivable cell. Belief updates are handled by clue_probability_field().
        self.recompute_value_map()
        print(f"Marked occupied clue cell: {cell}")
        return True

    def mark_obstacle(self, cell: Cell) -> bool:
        if self.map is None or not self.map.in_bounds(cell):
            return False
        i = self.map.idx(cell)
        already_obstacle = self.grid[i] == CELL_OBSTACLE
        self.grid[i] = CELL_OBSTACLE
        self.target_p[i] = 0.0
        self.clue_p[i] = 0.0
        self.obstacle_count += 1
        self.renorm(self.target_p)
        self.renorm(self.clue_p)
        self.recompute_value_map()
        print(f"Marked obstacle cell: {cell}")
        return not already_obstacle

    # =======================================================
    # Preserved Auction-Greedy goal selection / A*
    # =======================================================

    def pick_goal(self) -> Optional[GoalSelection]:
        assert self.map is not None
        assert self.my_cell is not None

        reserved_by_peers = {
            peer.goal_cell for peer in self.peers.values()
            if peer.goal_cell is not None
        }
        predicted_positions = {
            rid: peer.pos_cell or peer.goal_cell
            for rid, peer in self.peers.items()
            if (peer.pos_cell is not None or peer.goal_cell is not None)
        }

        best: Optional[GoalSelection] = None
        best_val = -1e18
        fallback_best: Optional[GoalSelection] = None
        fallback_val = -1e18

        def can_win(movement_cell: Cell, score: float) -> bool:
            value_before_my_travel = score + octile_cell(self.my_cell, movement_cell)
            for rid, start in predicted_positions.items():
                if start is None:
                    continue
                peer_score = value_before_my_travel - octile_cell(movement_cell, start)
                if peer_score > score:
                    return False
                if peer_score == score and rid < ROBOT_ID:
                    return False
            return True

        def consider_selection(selection: GoalSelection, score: float):
            nonlocal best, best_val, fallback_best, fallback_val
            movement_cell = selection.movement_cell
            if movement_cell not in reserved_by_peers or movement_cell == self.current_goal_cell:
                if score > fallback_val:
                    fallback_val = score
                    fallback_best = selection
            if movement_cell in reserved_by_peers:
                return
            if not can_win(movement_cell, score):
                return
            if score > best_val:
                best_val = score
                best = selection

        def consider_movement_cell(cell: Cell):
            if not self.map.in_bounds(cell):
                return
            if not self.is_traversable(cell):
                return
            if not is_search_goal_candidate(self.grid[self.map.idx(cell)]):
                return
            score = (self.prob_map[self.map.idx(cell)] * REWARD_FACTOR) - octile_cell(self.my_cell, cell)
            consider_selection(GoalSelection(movement_cell=cell), score)

        def consider_interrogation_target(target_cell: Cell):
            if not self.is_observation_target(target_cell):
                return
            target_value = self.prob_map[self.map.idx(target_cell)]

            # Unknown objects are not movement goals; they are observation
            # targets approached from a valid facing direction. Interrogation
            # goals are reached by choosing an approach path whose final heading
            # points from the observation cell directly toward the occupied
            # target cell. Navigation will not rotate after arrival; the camera
            # is assumed to face the final movement direction. This exact-facing
            # constraint can be relaxed later when the camera field of view is
            # known.
            for observation_cell in self.observation_cells_for(target_cell):
                if observation_cell in reserved_by_peers and observation_cell != self.current_goal_cell:
                    continue
                required_heading = heading_index_toward(observation_cell, target_cell)
                if required_heading is None:
                    continue
                path, path_cost = self.a_star_with_cost(
                    self.my_cell,
                    observation_cell,
                    required_goal_heading=required_heading,
                )
                if not path:
                    continue
                score = target_value + UNKNOWN_OBJECT_BONUS - path_cost
                consider_selection(
                    GoalSelection(
                        movement_cell=observation_cell,
                        interrogation_target=target_cell,
                        required_goal_heading=required_heading,
                    ),
                    score,
                )

        # Bias goal choice toward cells that are reachable by smooth immediate
        # steering from the current car-like heading.
        if self.heading_index is not None:
            hx, hy = DIRS8[self.heading_index]
            consider_movement_cell((self.my_cell[0] + hx, self.my_cell[1] + hy))
            if best is None:
                left = DIRS8[(self.heading_index - 1) % len(DIRS8)]
                right = DIRS8[(self.heading_index + 1) % len(DIRS8)]
                consider_movement_cell((self.my_cell[0] + left[0], self.my_cell[1] + left[1]))
                consider_movement_cell((self.my_cell[0] + right[0], self.my_cell[1] + right[1]))

        # Before any clue is seen, this still searches the grid by highest uniform
        # value with the auction cost. Later you can add explicit regional bands if desired.
        for y in range(self.map.height_cells):
            for x in range(self.map.width_cells):
                cell = (x, y)
                consider_movement_cell(cell)
                consider_interrogation_target(cell)

        if best is not None:
            return best
        if fallback_best is not None:
            return fallback_best

        unknowns = [
            (x, y)
            for y in range(self.map.height_cells)
            for x in range(self.map.width_cells)
            if is_search_goal_candidate(self.grid[self.map.idx((x, y))]) and (x, y) not in reserved_by_peers
        ]
        if unknowns:
            return GoalSelection(
                movement_cell=min(unknowns, key=lambda c: manhattan_cell(c, self.my_cell))
            )
        return None

    def is_traversable(self, cell: Cell) -> bool:
        assert self.map is not None
        return self.map.in_bounds(cell) and is_traversable_cell_state(self.grid[self.map.idx(cell)])

    def is_observation_target(self, cell: Cell) -> bool:
        assert self.map is not None
        if not self.map.in_bounds(cell):
            return False
        # Unknown LiDAR objects may be clues. They are high-value camera
        # interrogation targets, but they are never drivable movement goals.
        return self.grid[self.map.idx(cell)] == CELL_OCCUPIED_UNKNOWN

    def observation_cells_for(self, target_cell: Cell) -> List[Cell]:
        assert self.map is not None
        cells = []
        for neighbor in adjacent_cells_8(target_cell):
            if self.is_traversable(neighbor):
                cells.append(neighbor)
        return cells

    def a_star(
        self,
        start: Cell,
        goal: Cell,
        required_goal_heading: Optional[int] = None,
    ) -> List[Cell]:
        path, _ = self.a_star_with_cost(start, goal, required_goal_heading)
        return path

    def a_star_with_cost(
        self,
        start: Cell,
        goal: Cell,
        required_goal_heading: Optional[int] = None,
    ) -> Tuple[List[Cell], float]:
        """
        This planner approximates nonholonomic RC-car motion using heading-aware
        8-direction grid planning with constrained steering transitions.

        Normal movement goals accept any final heading. Interrogation goals may
        pass required_goal_heading so the final approach direction points from
        the observation cell directly toward the occupied target cell.
        The camera is assumed to look straight ahead along that final movement
        direction; this conservative exact-facing rule can be relaxed later
        once camera field of view is modeled.
        """
        assert self.map is not None
        n_headings = len(DIRS8)
        n_states = self.map.n_cells * n_headings
        frontier: List[Tuple[float, int]] = []
        came_from = [-1] * n_states
        cost_so_far = [float("inf")] * n_states

        start_idx = self.map.idx(start)
        goal_idx = self.map.idx(goal)

        start_headings = self.start_heading_candidates(start, goal)
        for heading_idx in start_headings:
            start_state = planner_state_index(start_idx, heading_idx)
            heapq.heappush(frontier, (0.0, start_state))
            came_from[start_state] = start_state
            cost_so_far[start_state] = 0.0

        temporary_peer_obstacles = self.dynamic_peer_obstacles()
        goal_state = -1

        while frontier and not self.found_target:
            _, current_state = heapq.heappop(frontier)
            current_idx, current_heading = planner_state_parts(current_state)
            if current_idx == goal_idx and (
                required_goal_heading is None or current_heading == required_goal_heading
            ):
                goal_state = current_state
                break

            current = self.map.cell_from_idx(current_idx)
            cx, cy = current

            # Neighbor generation is constrained to forward-left, forward, or
            # forward-right. This prevents zero-radius pivots and instant
            # reversals while retaining a lightweight grid planner.
            for steering_delta in STEERING_DELTAS:
                next_heading = (current_heading + steering_delta) % n_headings
                dx, dy = DIRS8[next_heading]
                nxt = (cx + dx, cy + dy)
                if not self.map.in_bounds(nxt):
                    continue
                ni = self.map.idx(nxt)
                # A* routes only through traversable cell states. Unsearched
                # cells are allowed as tentative drivable space because the
                # navigation process is expected to verify the cell before
                # entering it; occupied/obstacle states are never traversable.
                if not self.is_traversable(nxt):
                    continue
                # Peer positions / intents are temporary obstacles, like the old yield logic.
                if nxt in temporary_peer_obstacles and nxt != goal:
                    continue

                move_cost = movement_cost(dx, dy)
                # Turn-cost logic: straight motion is cheap, 45-degree steering
                # is moderate, and larger instantaneous heading changes are
                # very expensive or effectively blocked by the action set.
                turn_cost = turn_cost_for_delta(abs(steering_delta))
                visited_pen = VISITED_STEP_PENALTY if is_observed_empty(self.grid[ni]) else 0.0
                base_cost = move_cost + turn_cost + visited_pen

                reward_bonus = self.prob_map[ni] * REWARD_FACTOR
                reward_bonus = min(reward_bonus, max(0.0, base_cost - 0.01))
                step_cost = max(0.01, base_cost - reward_bonus)
                new_cost = cost_so_far[current_state] + step_cost
                next_state = planner_state_index(ni, next_heading)

                if new_cost < cost_so_far[next_state]:
                    cost_so_far[next_state] = new_cost
                    priority = new_cost + octile_cell(nxt, goal)
                    heapq.heappush(frontier, (priority, next_state))
                    came_from[next_state] = current_state

        if goal_state == -1:
            return [], float("inf")

        path: List[Cell] = []
        cur = goal_state
        while cur != came_from[cur]:
            cur_cell_idx, _ = planner_state_parts(cur)
            path.append(self.map.cell_from_idx(cur_cell_idx))
            cur = came_from[cur]
        path.reverse()
        return [start] + path, cost_so_far[goal_state]

    def start_heading_candidates(self, start: Cell, goal: Cell) -> List[int]:
        if self.heading_index is not None:
            return [self.heading_index]

        # If navigation has not reported heading_deg yet, choose an initial
        # planner heading toward the goal. Once Navigation reports heading_deg,
        # that authoritative NAV-PVT headMot value is used.
        goal_heading = heading_index_toward(start, goal)
        if goal_heading is not None:
            return [goal_heading]
        return list(range(len(DIRS8)))

    def dynamic_peer_obstacles(self) -> Set[Cell]:
        if SINGLE_ROBOT_MODE:
            return set()
        cells = set()
        for peer in self.peers.values():
            if peer.pos_cell is not None:
                cells.add(peer.pos_cell)
            if peer.intent_cell is not None:
                cells.add(peer.intent_cell)
        return cells

    def i_should_yield(self, cell: Cell) -> bool:
        return cell in self.dynamic_peer_obstacles()

    # =======================================================
    # Planning loop and outputs to navigation
    # =======================================================

    def plan_and_publish_if_needed(self, force: bool = False):
        if self.map is None or self.my_cell is None or self.found_target:
            return
        if self.interrogation_wait_started_at is not None:
            return
        if self.last_waypoint_cell is not None and not force:
            return

        log_task_event("planning_started", {"force": force, "my_cell": self.my_cell, "heading_index": self.heading_index})
        prev_goal = self.current_goal_cell
        selection = self.pick_goal()
        if selection is None:
            print("No available goal cells remain.")
            log_task_event("path_failed", {"reason": "no_available_goal", "my_cell": self.my_cell})
            self.publish_stop(reason="no_available_goal")
            return

        goal = selection.movement_cell
        interrogation_target = selection.interrogation_target
        required_goal_heading = selection.required_goal_heading
        if goal != prev_goal:
            self.goal_replan_count += 1
            self.current_goal = goal
            self.current_goal_cell = goal
            self.current_interrogation_target = interrogation_target
            self.current_required_goal_heading = required_goal_heading
            goal_lat, goal_lon = self.map.cell_to_gps(goal)
            self.publish_peer("goal", {"lat": goal_lat, "lon": goal_lon})
        else:
            self.current_interrogation_target = interrogation_target
            self.current_required_goal_heading = required_goal_heading
        log_task_event(
            "goal_selected",
            {
                "goal_cell": goal,
                "interrogation_target": interrogation_target,
                "required_goal_heading": required_goal_heading,
                "my_cell": self.my_cell,
            },
        )

        path = self.a_star(self.my_cell, goal, required_goal_heading=required_goal_heading)
        if len(path) == 1 and goal == self.my_cell:
            self.current_path = path
            log_task_event("path_planned", {"path": path, "goal_cell": goal, "my_cell": self.my_cell})
            self.publish_waypoint(self.my_cell, goal)
            return
        if len(path) < 2:
            self.clear_current_goal()
            self.current_path = []
            self.path_replan_count += 1
            log_task_event("path_failed", {"reason": "a_star_no_path", "goal_cell": goal, "my_cell": self.my_cell})
            return

        next_cell = path[1]
        if self.i_should_yield(next_cell):
            self.yield_count += 1
            self.path_replan_count += 1
            log_task_event("replan_triggered", {"reason": "yield_to_peer", "next_cell": next_cell})
            return

        self.current_path = path
        log_task_event("path_planned", {"path": path, "goal_cell": goal, "next_cell": next_cell, "my_cell": self.my_cell})
        self.publish_waypoint(next_cell, goal)

    def publish_waypoint(self, next_cell: Cell, goal_cell: Cell):
        assert self.map is not None
        lat, lon = self.map.cell_to_gps(next_cell)
        goal_lat, goal_lon = self.map.cell_to_gps(goal_cell)
        self.last_waypoint_cell = next_cell

        # Publish intent to peer as GPS, per your instruction.
        self.publish_peer("intent", {"lat": lat, "lon": lon})

        # USER-EDITABLE MESSAGE FORMAT: waypoint command to navigation
        # Navigation only executes waypoint driving. TaskManager is responsible
        # for choosing waypoints that leave the vehicle facing the correct cell
        # for camera interrogation, assuming the camera looks along the final
        # movement direction. Do not put interrogation mode/target/heading
        # requirements in this navigation command.
        payload = {
            "robot_id": ROBOT_ID,
            "command": "go_to_waypoint",
            "waypoint": {
                "lat": lat,
                "lon": lon,
                "cell_x": next_cell[0],   # included for debugging; nav may ignore
                "cell_y": next_cell[1],   # included for debugging; nav may ignore
            },
            "goal": {
                "lat": goal_lat,
                "lon": goal_lon,
                "cell_x": goal_cell[0],
                "cell_y": goal_cell[1],
            },
            "timestamp": time.time(),
        }
        self.client.publish(TOPIC_CMD_WAYPOINT, json.dumps(payload))
        log_task_event(
            "waypoint_command_published",
            {
                "current_robot_cell": self.my_cell,
                "next_waypoint_cell": next_cell,
                "goal_cell": goal_cell,
                "waypoint_lat": lat,
                "waypoint_lon": lon,
                "goal_lat": goal_lat,
                "goal_lon": goal_lon,
                "movement_type": "interrogation_observation" if self.current_interrogation_target is not None else "normal",
                "interrogation_target": self.current_interrogation_target,
                "current_heading_index": self.heading_index,
                "planned_path": self.current_path,
                "payload": payload,
            },
        )
        if self.current_interrogation_target is not None:
            print(
                f"Waypoint -> cell={next_cell}, gps=({lat:.7f}, {lon:.7f}), "
                f"goal={goal_cell}, internal_interrogation_target={self.current_interrogation_target}"
            )
        else:
            print(f"Waypoint -> cell={next_cell}, gps=({lat:.7f}, {lon:.7f}), goal={goal_cell}")

    def publish_stop(self, reason: str):
        payload = {
            "robot_id": ROBOT_ID,
            "command": "stop",
            "reason": reason,
            "timestamp": time.time(),
        }
        try:
            self.client.publish(TOPIC_CMD_STOP, json.dumps(payload))
            log_task_event("stop_published", payload)
        except Exception:
            log_task_error("stop_publish_failed", "publish failed", payload)

    def handle_target_found(self, lat: float, lon: float, source: str, clue_id: Optional[int]):
        if self.found_target:
            return
        self.found_target = True
        self.target_location_gps = (lat, lon)
        self.publish_stop(reason=f"target_found:{source}")
        self.publish_peer("target", {"lat": lat, "lon": lon, "clue_id": clue_id})
        log_task_event("target_found", {"lat": lat, "lon": lon, "source": source, "clue_id": clue_id})
        print(f"TARGET FOUND by {source}: clue_id={clue_id}, gps=({lat:.7f}, {lon:.7f})")


# ===========================================================
# Geometry helpers
# ===========================================================

EARTH_RADIUS_M = 6371000.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dp = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def meters_to_lat_delta(north_m: float) -> float:
    return math.degrees(north_m / EARTH_RADIUS_M)


def meters_to_lon_delta(east_m: float, at_lat: float) -> float:
    denom = EARTH_RADIUS_M * math.cos(math.radians(at_lat))
    if abs(denom) < 1e-9:
        return 0.0
    return math.degrees(east_m / denom)


def signed_north_m(origin_lat: float, origin_lon: float, lat: float) -> float:
    sign = 1.0 if lat >= origin_lat else -1.0
    return sign * haversine_m(origin_lat, origin_lon, lat, origin_lon)


def signed_east_m(origin_lat: float, origin_lon: float, lon: float) -> float:
    sign = 1.0 if lon >= origin_lon else -1.0
    return sign * haversine_m(origin_lat, origin_lon, origin_lat, lon)


def project_gps(lat: float, lon: float, bearing_deg: float, distance_m: float) -> GPS:
    bearing = math.radians(bearing_deg)
    lat1 = math.radians(lat)
    lon1 = math.radians(lon)
    dr = distance_m / EARTH_RADIUS_M
    lat2 = math.asin(
        math.sin(lat1) * math.cos(dr)
        + math.cos(lat1) * math.sin(dr) * math.cos(bearing)
    )
    lon2 = lon1 + math.atan2(
        math.sin(bearing) * math.sin(dr) * math.cos(lat1),
        math.cos(dr) - math.sin(lat1) * math.sin(lat2),
    )
    return (math.degrees(lat2), math.degrees(lon2))


def is_traversable_cell_state(state: int) -> bool:
    # CELL_UNSEARCHED remains tentatively traversable so the search planner can
    # enter new territory. Navigation must still verify free space before
    # physically entering the cell.
    return state in {CELL_UNSEARCHED, CELL_FREE_SEARCHED}


def is_observed_empty(state: int) -> bool:
    return state == CELL_FREE_SEARCHED


def is_search_goal_candidate(state: int) -> bool:
    # Waypoint goals must be traversable. Occupied-unknown cells are prioritized
    # indirectly through adjacent traversable cells for camera interrogation.
    return state == CELL_UNSEARCHED


def adjacent_cells_8(cell: Cell) -> List[Cell]:
    x, y = cell
    return [(x + dx, y + dy) for dx, dy in DIRS8]


def manhattan_cell(a: Cell, b: Cell) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def octile_cell(a: Cell, b: Cell) -> float:
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    diagonal = min(dx, dy)
    straight = max(dx, dy) - diagonal
    return straight + math.sqrt(2.0) * diagonal


def cell_direction(a: Cell, b: Cell) -> Optional[Tuple[int, int]]:
    dx = b[0] - a[0]
    dy = b[1] - a[1]
    if dx == 0 and dy == 0:
        return None
    return (
        0 if dx == 0 else (1 if dx > 0 else -1),
        0 if dy == 0 else (1 if dy > 0 else -1),
    )


def heading_index_from_vector(direction: Optional[Tuple[int, int]]) -> Optional[int]:
    if direction is None:
        return None
    try:
        return DIRS8.index(direction)
    except ValueError:
        return None


def heading_index_from_degrees(heading_deg: Optional[float]) -> Optional[int]:
    if heading_deg is None:
        return None
    try:
        normalized = float(heading_deg) % 360.0
    except (TypeError, ValueError):
        return None
    return int(round(normalized / 45.0)) % len(DIRS8)


def scan_heading_index(
    robot_heading_deg: Optional[float],
    direction: Optional[str],
    relative_heading_deg: Optional[float],
) -> Optional[int]:
    base_heading = heading_index_from_degrees(robot_heading_deg)
    if base_heading is None:
        return None

    if relative_heading_deg is not None:
        try:
            return heading_index_from_degrees(float(robot_heading_deg) + float(relative_heading_deg))
        except (TypeError, ValueError):
            return None

    direction_offsets = {
        "front_left": -1,
        "front": 0,
        "front_right": 1,
        "left": -2,
        "right": 2,
    }
    if direction not in direction_offsets:
        return None
    return (base_heading + direction_offsets[direction]) % len(DIRS8)


def heading_index_toward(a: Cell, b: Cell) -> Optional[int]:
    return heading_index_from_vector(cell_direction(a, b))


def movement_cost(dx: int, dy: int) -> float:
    if dx != 0 and dy != 0:
        return math.sqrt(2.0)
    return 1.0


def turn_cost_for_delta(delta_steps: int) -> float:
    """Return cost for an instantaneous heading change measured in 45-degree steps."""
    delta_steps = min(delta_steps, len(DIRS8) - delta_steps)
    if delta_steps == 0:
        return TURN_COST_STRAIGHT
    if delta_steps == 1:
        return TURN_COST_45_DEG
    if delta_steps == 2:
        return TURN_COST_90_DEG
    if delta_steps == 3:
        return TURN_COST_135_DEG
    return TURN_COST_180_DEG


def planner_state_index(cell_idx: int, heading_idx: int) -> int:
    return cell_idx * len(DIRS8) + heading_idx


def planner_state_parts(state_idx: int) -> Tuple[int, int]:
    return divmod(state_idx, len(DIRS8))


if __name__ == "__main__":
    manager = AuctionGreedyTaskManager()
    manager.start()
