"""Radar-based risk evaluator using the original grise thresholds and hold behavior.
The numerical thresholds are carried over as initial values and must be re-tuned after real radar tests.
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
import config
from perception.radar import RadarTarget

_LEVEL_ORDER = {"SAFE": 0, "WARN": 1, "DANGER": 2}


@dataclass
class RiskState:
    level: str = "SAFE"
    distance_cm: float | None = None
    approach_speed_cm_s: float = 0.0
    ttc_s: float | None = None
    reason: str = "no radar target"
    threat_direction: str | None = None
    timestamp: float | None = None


def classify_target(distance_cm: float, approach_speed_cm_s: float) -> tuple[str, str, float | None]:
    """Existing distance/TTC thresholds, evaluated on one normalized target."""
    ttc = distance_cm / approach_speed_cm_s if approach_speed_cm_s > 0 else None
    if distance_cm < config.RISK_DANGER_DIST_CM:
        return "DANGER", "DISTANCE", ttc
    if (approach_speed_cm_s > config.RISK_APPROACH_SPEED_CM_S and
            ttc is not None and ttc < config.RISK_TTC_DANGER_S):
        return "DANGER", "TTC", ttc
    if distance_cm < config.RISK_WARN_DIST_CM:
        return "WARN", "DISTANCE", ttc
    if (approach_speed_cm_s > config.RISK_APPROACH_SPEED_CM_S and
            ttc is not None and ttc < config.RISK_TTC_WARN_S):
        return "WARN", "TTC", ttc
    return "SAFE", "normal", ttc


class RiskEvaluator:
    def __init__(self) -> None:
        self._dist_ema: float | None = None
        self._speed_ema = 0.0
        self._held_level = "SAFE"
        self._held_until = 0.0
        self._last_id: int | None = None
        self._last_distance: float | None = None
        self._last_angle: float | None = None

    def reset(self) -> None:
        self._dist_ema = None
        self._speed_ema = 0.0
        self._last_id = None
        self._last_distance = None
        self._last_angle = None

    def invalidate(self, reason: str, now: float | None = None) -> RiskState:
        self.reset()
        self._held_level = "SAFE"
        self._held_until = 0.0
        return RiskState("INVALID", reason=reason, timestamp=time.monotonic() if now is None else now)

    def evaluate(self, target: RadarTarget | None, now: float | None = None) -> RiskState:
        now = time.monotonic() if now is None else now
        direction = None
        if hasattr(target, "radar_valid"):
            world = target
            if not world.sensor_valid or not world.vision_valid or not world.radar_valid:
                return self.invalidate("sensor invalid", now)
            direction = world.threat.direction if world.threat else None
            target = world.radar_target
        if target is None:
            self.reset()
            return self._apply_hold(RiskState(reason="no radar target", timestamp=now), now)

        if not all(math.isfinite(v) for v in (target.distance_cm, target.angle_deg,
                                               target.approach_speed_cm_s)) or target.distance_cm < 0:
            return self.invalidate("invalid radar target", now)
        switched = ((target.target_id is not None and self._last_id != target.target_id) or
                    (target.target_id is None and self._last_distance is not None and
                     (abs(target.distance_cm - self._last_distance) > config.RADAR_TARGET_SWITCH_DISTANCE_CM or
                      abs(math.remainder(target.angle_deg - self._last_angle, 360)) >
                      config.RADAR_TARGET_SWITCH_ANGLE_DEG)))
        if switched:
            self.reset()
            self._held_level = "SAFE"
            self._held_until = 0.0
        self._last_id = target.target_id
        self._last_distance = target.distance_cm
        self._last_angle = target.angle_deg

        ad = config.RISK_DIST_EMA_ALPHA
        av = config.RISK_SPEED_EMA_ALPHA
        self._dist_ema = target.distance_cm if self._dist_ema is None else (1-ad)*self._dist_ema + ad*target.distance_cm
        speed = max(0.0, target.approach_speed_cm_s) if target.approaching else 0.0
        self._speed_ema = (1-av)*self._speed_ema + av*speed if target.approaching else 0.0
        d, v = self._dist_ema, self._speed_ema
        smoothed_level, smoothed_reason, _ = classify_target(d, v)
        raw_level, raw_reason, raw_ttc = classify_target(target.distance_cm, speed)
        level, reason = ((raw_level, raw_reason) if _LEVEL_ORDER[raw_level] >=
                         _LEVEL_ORDER[smoothed_level] else (smoothed_level, smoothed_reason))
        return self._apply_hold(RiskState(level, d, v, raw_ttc, reason, direction, now), now)

    def _apply_hold(self, state: RiskState, now: float) -> RiskState:
        new_rank = _LEVEL_ORDER[state.level]
        held_rank = _LEVEL_ORDER[self._held_level]
        if new_rank >= held_rank:
            self._held_level = state.level
            hold = config.RISK_DANGER_HOLD_S if state.level == "DANGER" else config.RISK_WARN_HOLD_S if state.level == "WARN" else 0.0
            self._held_until = now + hold
        elif now < self._held_until:
            state.level = self._held_level
            state.reason += " (hold)"
        else:
            self._held_level = state.level
        return state
