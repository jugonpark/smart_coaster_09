"""One bounded escape action per ESP32 motion ID."""
from __future__ import annotations

import secrets
import math

import config


class MotionGoals:
    def __init__(self, initial_id: int | None = None) -> None:
        self._next_id = initial_id if initial_id is not None else secrets.randbelow(2**30)
        self._boot_id = None
        self._active_id = None
        self._key = None
        self._completed_revision = None
        self._needs_stop = True

    def select(self, command, plan, telemetry, revision: int):
        """Return (motion_id, distance), or None to transmit STOP."""
        if command.status == "STOP":
            self._active_id = None
            self._key = None
            return None
        if (not telemetry.received or telemetry.age_s is None or
                not math.isfinite(telemetry.age_s) or telemetry.age_s < 0 or
                telemetry.age_s > config.TELEMETRY_STALE_S or
                telemetry.boot_id is None):
            self._active_id = None
            self._needs_stop = True
            return None
        if telemetry.boot_id != self._boot_id:
            self._boot_id = telemetry.boot_id
            self._active_id = None
            self._needs_stop = True
        if self._needs_stop:
            self._needs_stop = False
            return None
        if (self._active_id is not None and
                telemetry.motion_id == self._active_id and telemetry.goal_reached):
            self._active_id = None
            self._completed_revision = revision
            return None
        if (self._active_id is not None and telemetry.motion_id == self._active_id and
                not telemetry.goal_active and telemetry.status == "STOP"):
            self._active_id = None
            self._needs_stop = True
            self._completed_revision = revision
            return None
        if self._completed_revision is not None:
            if revision <= self._completed_revision:
                return None
            self._completed_revision = None
        key = (plan.direction, plan.target_distance_cm, command.status)
        if self._active_id is None or key != self._key:
            self._next_id = (self._next_id % (2**31 - 1)) + 1
            self._active_id = self._next_id
            self._key = key
        return self._active_id, plan.target_distance_cm
