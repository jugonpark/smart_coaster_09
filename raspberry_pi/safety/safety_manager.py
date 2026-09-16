from __future__ import annotations

from dataclasses import dataclass
import math
import time
import config
from decision.risk_evaluator import RiskState
from planning.escape_planner import EscapePlan
from perception.vision_state import DIRECTIONS


@dataclass
class MotionCommand:
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    status: str = "STOP"
    reason: str = "safe stop"


class SafetyManager:
    """Final authority before a motion command reaches ESP32."""

    def validate(self, *, camera_ok: bool, world, plan: EscapePlan, telemetry,
                 risk: RiskState | None = None, now: float | None = None) -> MotionCommand:
        now = time.monotonic() if now is None else now
        if not camera_ok:
            return MotionCommand(reason="camera offline")
        if not world.robot.detected:
            return MotionCommand(reason="robot ArUco lost")
        if not world.sensor_valid:
            return MotionCommand(reason="sensor state invalid")
        if risk is not None and risk.level == "INVALID":
            return MotionCommand(reason="risk invalid")
        modern = world.vision is not None
        if modern:
            if not world.vision_valid or not world.vision.camera_ok or not world.vision.detector_available:
                return MotionCommand(reason="vision invalid")
            if not world.radar_valid:
                return MotionCommand(reason="radar invalid")
            if (world.timestamp is None or not math.isfinite(world.timestamp) or
                    world.timestamp > now or now - world.timestamp > config.WORLD_STALE_S):
                return MotionCommand(reason="WorldState stale")
        if not world.radar_connected and not config.ALLOW_MOTION_WITHOUT_RADAR:
            return MotionCommand(reason="radar disconnected")
        if config.REQUIRE_TELEMETRY_FOR_MOTION:
            if (not telemetry.received or telemetry.age_s is None or
                    not math.isfinite(telemetry.age_s) or telemetry.age_s < 0 or
                    telemetry.age_s > config.TELEMETRY_STALE_S):
                return MotionCommand(reason="ESP32 telemetry stale")
        if plan.status == "STOP":
            return MotionCommand(reason=plan.reason)
        if modern:
            expected_status = {"WARN": "SLOW", "DANGER": "RUN"}.get(
                risk.level if risk is not None else None)
            if (not plan.valid or expected_status != plan.status or
                    plan.timestamp is None or not math.isfinite(plan.timestamp) or
                    plan.timestamp > now or now - plan.timestamp > config.WORLD_STALE_S or
                    plan.direction not in DIRECTIONS or plan.direction not in world.sectors or
                    world.sectors[plan.direction] != "CLEAR" or
                    not math.isfinite(plan.speed_cm_s) or plan.speed_cm_s <= 0 or
                    not math.isfinite(plan.target_distance_cm) or plan.target_distance_cm <= 0 or
                    not math.isclose(math.hypot(plan.vx, plan.vy), plan.speed_cm_s,
                                     abs_tol=1e-6) or
                    not math.isclose(plan.vx, plan.speed_cm_s * math.cos(
                        math.radians(DIRECTIONS.index(plan.direction) * 45)), abs_tol=1e-6) or
                    not math.isclose(plan.vy, plan.speed_cm_s * math.sin(
                        math.radians(DIRECTIONS.index(plan.direction) * 45)), abs_tol=1e-6)):
                return MotionCommand(reason="EscapePlan invalid")
        if (plan.status not in ("RUN", "SLOW") or
                not all(math.isfinite(x) for x in (plan.vx, plan.vy, plan.w)) or
                math.hypot(plan.vx, plan.vy) > config.MAX_LINEAR_SPEED_CM_S or
                abs(plan.w) > config.MAX_ANGULAR_SPEED_RAD_S):
            return MotionCommand(reason="invalid motion command")
        if not world.direction_clear(plan.direction):
            return MotionCommand(reason=f"escape direction blocked: {plan.direction}")
        if not config.ENABLE_ESCAPE_MOTION:
            return MotionCommand(reason=f"motion disabled; planned {plan.direction} {plan.target_distance_cm:.0f}cm")
        return MotionCommand(plan.vx, plan.vy, plan.w, plan.status, plan.reason)
