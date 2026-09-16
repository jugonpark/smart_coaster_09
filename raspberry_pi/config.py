"""GRISE Raspberry Pi central-control configuration.

Source-derived defaults are carried over from the provided grise-master where possible.
New values added for the 3-node architecture are explicitly marked as NEW.
"""
from __future__ import annotations
import os

# ----- Laptop network camera -----
# The old ESP32 sketch used 10.182.7.110 as its gateway. That does NOT prove it is
# the laptop address, so verify the actual laptop IP before running.
LAPTOP_IP = os.getenv("GRISE_LAPTOP_IP", "10.182.7.110")
LAPTOP_CAMERA_PORT = 8080
NETWORK_CAMERA_URL = os.getenv(
    "GRISE_CAMERA_URL", f"http://{LAPTOP_IP}:{LAPTOP_CAMERA_PORT}/stream.mjpg"
)
FRAME_WIDTH = 1280
FRAME_HEIGHT = 720
TARGET_FPS = 30
FLIP_HORIZONTAL = False
DEFAULT_PX_PER_CM = 8.0

# ----- YOLO (carried over) -----
YOLO_BACKEND = os.getenv("GRISE_YOLO_BACKEND", "ultralytics")  # ultralytics | roboflow | stub
YOLO_WEIGHTS = os.getenv("GRISE_YOLO_WEIGHTS", "models/dig_yolo.pt")
ROBOFLOW_MODEL_ID = "your-workspace/your-project/1"
ROBOFLOW_API_KEY = ""
YOLO_CONF_THRESHOLD = 0.45
YOLO_IMG_SIZE = 640
YOLO_EVERY_N_FRAMES = 2
CLASS_CUP = "cup"
CLASS_OBSTACLE_NAMES = ("obstacle", "box", "bottle", "chair")

# ----- ArUco robot tracking (carried over) -----
ARUCO_DICT_NAME = "DICT_4X4_50"
ROBOT_MARKER_ID = 0
MARKER_SIZE_CM = 8.0
MARKER_HEADING_OFFSET_DEG = 0.0
ROBOT_POSE_EMA_ALPHA = 0.5

# ----- Radar interface -----
# NEW: radar integration is intentionally disabled in this handoff skeleton.
RADAR_ENABLED = os.getenv("GRISE_RADAR_ENABLED", "0") == "1"
RADAR_BACKEND = os.getenv("GRISE_RADAR_BACKEND", "")  # disabled | udp_fake | replay | serial
RADAR_REPLAY_PATH = os.getenv("GRISE_RADAR_REPLAY_PATH", "")
RADAR_BIND_HOST = "0.0.0.0"
RADAR_UDP_PORT = 8890
RADAR_STALE_S = 0.35
VISION_RADAR_MAX_SKEW_S = 0.35  # receive-time difference; not hardware synchronization
RADAR_TARGET_SWITCH_DISTANCE_CM = 30.0  # reset ID-less EMA on a large jump
RADAR_TARGET_SWITCH_ANGLE_DEG = 45.0
RADAR_MOUNT_YAW_DEG = 0.0  # assumption only; Project F handles alignment
RADAR_MAX_DISTANCE_CM = 1000.0  # provisional fake-input guard, not sensor capability
RADAR_MAX_RADIAL_SPEED_CM_S = 500.0  # provisional fake-input guard
RADAR_MAX_TARGETS = 128
RADAR_MAX_PACKET_BYTES = 4096
RADAR_MAX_PACKETS_PER_READ = 32
RADAR_RECONNECT_S = 1.0

# ----- Risk thresholds (carried over, initial values only; tune for radar) -----
RISK_DANGER_DIST_CM = 15.0
RISK_WARN_DIST_CM = 35.0
RISK_APPROACH_SPEED_CM_S = 25.0
RISK_TTC_DANGER_S = 0.8
RISK_TTC_WARN_S = 1.6
RISK_DIST_EMA_ALPHA = 0.4
RISK_SPEED_EMA_ALPHA = 0.3
RISK_DANGER_HOLD_S = 0.7
RISK_WARN_HOLD_S = 0.4

# ----- Motion limits (carried over) -----
MAX_LINEAR_SPEED_CM_S = 35.0
MAX_ANGULAR_SPEED_RAD_S = 1.8
MAX_LINEAR_ACCEL_CM_S2 = 90.0
HEADING_KP = 2.0
HEADING_DEADBAND_RAD = 0.08

# ----- Escape planner (NEW initial MVP values from current design discussion) -----
ESCAPE_DISTANCE_WARN_CM = 15.0
ESCAPE_DISTANCE_DANGER_CM = 30.0
ESCAPE_SPEED_WARN_CM_S = 15.0
ESCAPE_SPEED_DANGER_CM_S = 25.0
COLLISION_CHECK_DISTANCE_CM = 45.0
ROBOT_RADIUS_CM = 9.0  # initial chassis envelope; verify against physical robot
COLLISION_SAFETY_MARGIN_CM = 10.0

# Keep automatic motion disabled until camera->Pi->ESP32 and radar paths are verified.
ENABLE_ESCAPE_MOTION = os.getenv("GRISE_ENABLE_ESCAPE_MOTION", "0") == "1"
ALLOW_MOTION_WITHOUT_RADAR = False

# ----- Pi -> ESP32 command UDP (carried over) -----
ESP32_IP = os.getenv("GRISE_ESP32_IP", "10.182.7.50")
ESP32_PORT = 8888
UDP_SEND_HZ = 30

# ----- ESP32 -> Pi telemetry (NEW) -----
TELEMETRY_BIND_HOST = "0.0.0.0"
TELEMETRY_PORT = 8889
TELEMETRY_STALE_S = 0.7
REQUIRE_TELEMETRY_FOR_MOTION = True

# A WorldState is dated at frame acquisition, before potentially slow inference.
WORLD_STALE_S = 0.35

# ----- Pi -> laptop monitor (NEW) -----
MONITOR_IP = os.getenv("GRISE_MONITOR_IP", LAPTOP_IP)
MONITOR_PORT = 9001
MONITOR_SEND_HZ = 5

PRINT_HZ = 2.0
