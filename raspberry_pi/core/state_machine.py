from __future__ import annotations

import math

import config
from core.shared_state import SystemHealth, SystemSnapshot


class ControlStateMachine:
    """Gate control before evaluating risk or emitting a motion command."""

    def assess(self, state: SystemSnapshot, *, now: float) -> SystemHealth:
        if state.failure:
            return SystemHealth("FAULT", state.failure, now)
        if not state.camera_ok:
            return SystemHealth("STOP", "camera offline", now)
        if state.world is None or state.world_at is None:
            return SystemHealth("STOP", "WorldState unavailable", now)
        if not math.isfinite(state.world_at) or state.world_at > now:
            return SystemHealth("FAULT", "invalid WorldState timestamp", now)
        if now - state.world_at > config.WORLD_STALE_S:
            return SystemHealth("STALE", "WorldState stale", now)
        world = state.world
        if (not isinstance(world.sensor_valid, bool) or not world.sensor_valid or
                not all(math.isfinite(x) for x in (
                    world.robot.x_cm, world.robot.y_cm, world.robot.heading_rad
                ))):
            return SystemHealth("FAULT", "invalid WorldState", now)
        return SystemHealth("READY", "current WorldState", now)
