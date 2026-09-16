from __future__ import annotations

import math
import time
from dataclasses import dataclass

import config
from decision.risk_evaluator import RiskState
from fusion.sensor_fusion import WorldState
from perception.vision_state import DIRECTIONS, sector_for


@dataclass
class EscapePlan:
    vx: float = 0.0
    vy: float = 0.0
    w: float = 0.0
    status: str = "STOP"
    direction: str = "NONE"
    target_distance_cm: float = 0.0
    reason: str = "idle"
    valid: bool = False
    timestamp: float | None = None
    risk_level: str = "INVALID"
    threat_direction: str | None = None
    speed_cm_s: float = 0.0
    candidate_directions: tuple[str, ...] = ()
    blocked_directions: tuple[str, ...] = ()


_DIRS = {
    "FRONT": 0.0,
    "FRONT_LEFT": 45.0,
    "LEFT": 90.0,
    "BACK_LEFT": 135.0,
    "BACK": 180.0,
    "BACK_RIGHT": -135.0,
    "RIGHT": -90.0,
    "FRONT_RIGHT": -45.0,
}


class EscapePlanner:
    """Generate a body-frame escape velocity opposite to the radar threat."""

    def plan(self, world: WorldState, risk: RiskState,
             *, now: float | None = None) -> EscapePlan:
        now = time.monotonic() if now is None else now
        base = dict(timestamp=now, risk_level=risk.level)
        if risk.level == "SAFE":
            return EscapePlan(reason="no escape needed", **base)
        if risk.level not in ("WARN", "DANGER"):
            return EscapePlan(reason="risk invalid", **base)
        if (not world.sensor_valid or not world.robot.detected or
                not all(math.isfinite(v) for v in (
                    world.robot.x_cm, world.robot.y_cm, world.robot.heading_rad))):
            return EscapePlan(reason="sensor state invalid", **base)
        modern = world.vision is not None
        if modern:
            if (not world.vision_valid or not world.radar_valid or
                    world.timestamp is None or not math.isfinite(world.timestamp) or
                    world.timestamp > now or now - world.timestamp > config.WORLD_STALE_S or
                    set(world.sectors) != set(DIRECTIONS) or
                    any(value not in ("CLEAR", "BLOCKED", "UNKNOWN")
                        for value in world.sectors.values())):
                return EscapePlan(reason="WorldState invalid or stale", **base)
            if (risk.timestamp is not None and
                    (not math.isfinite(risk.timestamp) or risk.timestamp > now or
                     now - risk.timestamp > config.WORLD_STALE_S)):
                return EscapePlan(reason="RiskState stale", **base)
            threat_direction = world.threat.direction
        else:
            threat_direction = (sector_for(math.radians(world.radar_target.angle_deg))
                                if world.radar_target is not None else None)
        if threat_direction not in DIRECTIONS or world.radar_target is None:
            return EscapePlan(reason="threat direction unavailable", **base)

        preferred_index = (DIRECTIONS.index(threat_direction) + 4) % 8
        offsets = (0, -1, 1, -2, 2, -3, 3, 4)
        candidates = tuple(DIRECTIONS[(preferred_index + offset) % 8] for offset in offsets)
        blocked = tuple(direction for direction in candidates if
                        (world.sectors.get(direction) != "CLEAR" if modern else
                         not world.direction_clear(direction)))
        chosen = next((direction for direction in candidates if direction not in blocked), None)
        details = dict(**base, threat_direction=threat_direction,
                       candidate_directions=candidates, blocked_directions=blocked)
        if chosen is None:
            return EscapePlan(reason="all escape sectors blocked or unknown", **details)

        if risk.level == "DANGER":
            speed = config.ESCAPE_SPEED_DANGER_CM_S
            dist = config.ESCAPE_DISTANCE_DANGER_CM
            status = "RUN"
        else:
            speed = config.ESCAPE_SPEED_WARN_CM_S
            dist = config.ESCAPE_DISTANCE_WARN_CM
            status = "SLOW"

        angle = math.radians(_DIRS[chosen])
        vx = speed * math.cos(angle)
        vy = speed * math.sin(angle)
        return EscapePlan(
            vx=0.0 if abs(vx) < 1e-12 else vx,
            vy=0.0 if abs(vy) < 1e-12 else vy,
            w=0.0,
            status=status,
            direction=chosen,
            target_distance_cm=dist,
            reason=f"escape opposite {threat_direction}",
            valid=True, speed_cm_s=speed, **details,
        )
