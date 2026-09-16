"""Camera-only description of the robot and its surroundings.

No radar, risk level or motion decision belongs in this module.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import config
from perception.object_detector import Detection
from perception.robot_tracker import RobotPose
from planning.transform import world_to_robot


DIRECTIONS = ("FRONT", "FRONT_LEFT", "LEFT", "BACK_LEFT", "BACK",
              "BACK_RIGHT", "RIGHT", "FRONT_RIGHT")


@dataclass(frozen=True)
class RelativeObject:
    detection: Detection
    x_cm: float
    y_cm: float
    distance_cm: float
    bearing_rad: float
    timestamp: float


@dataclass
class VisionState:
    camera_ok: bool
    timestamp: float
    robot: RobotPose
    detector_available: bool
    objects: list[RelativeObject] = field(default_factory=list)
    sectors: dict[str, str] = field(default_factory=lambda: dict.fromkeys(DIRECTIONS, "UNKNOWN"))


def sector_for(bearing_rad: float) -> str:
    if not math.isfinite(bearing_rad):
        raise ValueError("non-finite bearing")
    # Exact half-sector boundaries belong to the counterclockwise sector.
    index = math.floor((math.degrees(bearing_rad) + 22.5) / 45.0) % 8
    return DIRECTIONS[index]


def build_vision_state(*, camera_ok: bool, captured_at: float, robot: RobotPose,
                       detections: list[Detection], detector_available: bool,
                       now: float) -> VisionState:
    state = VisionState(camera_ok, captured_at, robot, detector_available)
    fresh = (math.isfinite(captured_at) and captured_at <= now and
             now - captured_at <= config.WORLD_STALE_S)
    pose_at = robot.timestamp if robot.timestamp is not None else captured_at
    pose_valid = (robot.detected and math.isfinite(pose_at) and pose_at <= now and
                  now - pose_at <= config.WORLD_STALE_S and
                  all(math.isfinite(v) for v in (robot.x_cm, robot.y_cm, robot.heading_rad)))
    if not camera_ok or not fresh or not pose_valid:
        state.robot = RobotPose()
        return state
    if not detector_available:
        return state

    sectors = dict.fromkeys(DIRECTIONS, "CLEAR")
    try:
        for obj in detections:
            if not isinstance(obj, Detection):
                raise ValueError("invalid detection")
            values = (obj.x_cm, obj.y_cm, obj.radius_cm, obj.confidence,
                      obj.cx_px, obj.cy_px, obj.w_px, obj.h_px)
            if (not all(math.isfinite(v) for v in values)
                    or obj.radius_cm < 0 or obj.w_px < 0 or obj.h_px < 0
                    or not 0 <= obj.confidence <= 1):
                raise ValueError("invalid detection")
            detected_at = obj.timestamp if obj.timestamp is not None else captured_at
            if (not math.isfinite(detected_at) or detected_at > now or
                    now - detected_at > config.WORLD_STALE_S):
                raise ValueError("stale detection")
            x, y = world_to_robot(obj.x_cm - robot.x_cm,
                                  obj.y_cm - robot.y_cm, robot.heading_rad)
            distance = math.hypot(x, y)
            bearing = math.atan2(y, x)
            state.objects.append(RelativeObject(obj, x, y, distance, bearing, detected_at))
            if obj.role == "obstacle" and (distance - obj.radius_cm - config.ROBOT_RADIUS_CM
                                           <= config.COLLISION_CHECK_DISTANCE_CM):
                sectors[sector_for(bearing)] = "BLOCKED"
    except (AttributeError, TypeError, ValueError, OverflowError):
        state.objects.clear()
        state.detector_available = False
        return state
    state.sectors = sectors
    return state
