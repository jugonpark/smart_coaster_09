from __future__ import annotations

import math
from dataclasses import dataclass, field

import config
from perception.object_detector import Detection
from perception.robot_tracker import RobotPose
from perception.vision_state import VisionState
from perception.radar import RadarState, RadarTarget
from planning.transform import world_to_robot
from perception.vision_state import sector_for, DIRECTIONS
from decision.risk_evaluator import classify_target


@dataclass(frozen=True)
class ThreatState:
    detected: bool = False  # approaching target, distinct from any radar target
    direction: str | None = None
    distance_cm: float | None = None
    approach_speed_cm_s: float = 0.0
    approaching: bool = False
    ttc_s: float | None = None
    target_id: int | None = None
    target: RadarTarget | None = None


@dataclass
class SectorState:
    front: bool = False
    front_left: bool = False
    left: bool = False
    back_left: bool = False
    rear: bool = False
    back_right: bool = False
    right: bool = False
    front_right: bool = False


@dataclass
class WorldState:
    robot: RobotPose
    radar_connected: bool
    radar_target: RadarTarget | None
    obstacles: list[Detection] = field(default_factory=list)
    blocked: SectorState = field(default_factory=SectorState)
    sensor_valid: bool = True
    vision: VisionState | None = None
    timestamp: float | None = None
    vision_valid: bool = False
    radar_valid: bool = False
    radar_status: str = "OFFLINE"
    vision_age_s: float | None = None
    radar_age_s: float | None = None
    sectors: dict[str, str] = field(default_factory=dict)
    threat: ThreatState = field(default_factory=ThreatState)
    radar_targets: tuple[RadarTarget, ...] = ()

    @property
    def approaching_target_present(self) -> bool:
        return self.radar_valid and any(target.approaching for target in self.radar_targets)

    def direction_clear(self, name: str) -> bool:
        b = self.blocked
        table = {
            "FRONT": not b.front,
            "LEFT": not b.left,
            "RIGHT": not b.right,
            "BACK": not b.rear,
            "FRONT_LEFT": not b.front_left,
            "FRONT_RIGHT": not b.front_right,
            "BACK_LEFT": not b.back_left,
            "BACK_RIGHT": not b.back_right,
        }
        return table.get(name, False)


class SensorFusion:
    """MVP fusion: radar threat + camera/YOLO obstacle sectors around tracked robot."""

    def build(self, robot: RobotPose, obstacles: list[Detection], radar: RadarState,
              *, perception_available: bool = True) -> WorldState:
        blocked = SectorState()
        valid = perception_available and all(math.isfinite(x) for x in (
            robot.x_cm, robot.y_cm, robot.heading_rad
        ))
        if radar.target is not None:
            valid = valid and all(math.isfinite(x) for x in (
                radar.target.distance_cm, radar.target.angle_deg, radar.target.approach_speed_cm_s
            )) and radar.target.distance_cm >= 0
        if robot.detected:
            for o in obstacles:
                if not all(math.isfinite(x) for x in (o.x_cm, o.y_cm, o.radius_cm)) or o.radius_cm < 0:
                    valid = False
                    continue
                dx_w, dy_w = o.x_cm - robot.x_cm, o.y_cm - robot.y_cm
                dx_r, dy_r = world_to_robot(dx_w, dy_w, robot.heading_rad)
                center_dist = math.hypot(dx_r, dy_r)
                surface_dist = center_dist - max(o.radius_cm, 0.0)
                if surface_dist > config.COLLISION_CHECK_DISTANCE_CM:
                    continue
                angle = math.degrees(math.atan2(dy_r, dx_r))
                if -22.5 <= angle < 22.5:
                    blocked.front = True
                elif 22.5 <= angle < 67.5:
                    blocked.front_left = True
                elif 67.5 <= angle < 112.5:
                    blocked.left = True
                elif 112.5 <= angle < 157.5:
                    blocked.back_left = True
                elif -112.5 <= angle < -67.5:
                    blocked.right = True
                elif -67.5 <= angle < -22.5:
                    blocked.front_right = True
                elif -157.5 <= angle < -112.5:
                    blocked.back_right = True
                else:
                    blocked.rear = True
        return WorldState(robot, radar.connected, radar.target, obstacles, blocked, valid)

    def build_from_states(self, vision: VisionState, radar: RadarState, *, now: float) -> WorldState:
        """Fuse already-perceived snapshots; no detection, tracking, or escape planning."""
        vision_age = now - vision.timestamp if math.isfinite(vision.timestamp) else None
        radar_age = (now - radar.timestamp if radar.timestamp is not None and
                     math.isfinite(radar.timestamp) else None)
        pose = vision.robot
        pose_at = pose.timestamp if pose.timestamp is not None else vision.timestamp
        vision_valid = (vision.camera_ok and vision.detector_available and pose.detected and
                        vision_age is not None and 0 <= vision_age <= config.WORLD_STALE_S and
                        math.isfinite(pose_at) and 0 <= now - pose_at <= config.WORLD_STALE_S and
                        all(math.isfinite(v) for v in (pose.x_cm, pose.y_cm, pose.heading_rad)) and
                        set(vision.sectors) == set(DIRECTIONS) and
                        all(v in ("CLEAR", "BLOCKED", "UNKNOWN") for v in vision.sectors.values()))
        targets = radar.targets
        # The E receiver owns the transport health. F independently checks its snapshot.
        radar_valid = (radar.connected and radar.valid and radar.status in
                       ("NO_TARGET", "TARGET_DETECTED") and radar_age is not None and
                       0 <= radar_age <= config.RADAR_STALE_S and
                       ((radar.status == "NO_TARGET" and not targets and radar.target is None) or
                        (radar.status == "TARGET_DETECTED" and bool(targets))))
        if radar_valid:
            for target in targets:
                values = (target.distance_cm, target.angle_deg, target.approach_speed_cm_s,
                          target.radial_velocity_cm_s)
                if (not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values) or
                        not 0 <= target.distance_cm <= config.RADAR_MAX_DISTANCE_CM or
                        not -180 <= target.angle_deg <= 180 or
                        abs(target.approach_speed_cm_s) > config.RADAR_MAX_RADIAL_SPEED_CM_S or
                        target.approaching != (target.approach_speed_cm_s > 0) or
                        not math.isclose(target.radial_velocity_cm_s, -target.approach_speed_cm_s,
                                         abs_tol=1e-6) or
                        target.timestamp is None or not math.isfinite(target.timestamp) or
                        target.timestamp > now or now - target.timestamp > config.RADAR_STALE_S):
                    radar_valid = False
                    break
        if (vision_valid and radar_valid and
                abs(vision.timestamp - radar.timestamp) > config.VISION_RADAR_MAX_SKEW_S):
            radar_valid = False

        sectors = dict(vision.sectors) if vision_valid else dict.fromkeys(DIRECTIONS, "UNKNOWN")
        blocked = SectorState(**{
            attr: sectors[direction] != "CLEAR" for direction, attr in (
                ("FRONT", "front"), ("FRONT_LEFT", "front_left"), ("LEFT", "left"),
                ("BACK_LEFT", "back_left"), ("BACK", "rear"), ("BACK_RIGHT", "back_right"),
                ("RIGHT", "right"), ("FRONT_RIGHT", "front_right"))
        })
        primary = None
        threat = ThreatState()
        if radar_valid and targets:
            def rank(item):
                target = item[1]
                level, _, ttc = classify_target(target.distance_cm,
                    target.approach_speed_cm_s if target.approaching else 0.0)
                return ({"SAFE": 0, "WARN": 1, "DANGER": 2}[level],
                        -(ttc if ttc is not None else math.inf), -target.distance_cm, -item[0])
            primary = max(enumerate(targets), key=rank)[1]
            angle = math.radians(primary.angle_deg + config.RADAR_MOUNT_YAW_DEG)
            direction = sector_for(angle)
            approach = primary.approach_speed_cm_s if primary.approaching else 0.0
            threat = ThreatState(primary.approaching, direction, primary.distance_cm,
                                 approach, primary.approaching,
                                 primary.distance_cm / approach if approach > 0 else None,
                                 primary.target_id, primary)
        obstacles = [item.detection for item in vision.objects if item.detection.role == "obstacle"]
        return WorldState(pose, radar.connected and radar_valid, primary, obstacles,
                          blocked, vision_valid and radar_valid, vision,
                          now, vision_valid, radar_valid, radar.status,
                          vision_age, radar_age, sectors, threat,
                          tuple(targets) if radar_valid else ())
