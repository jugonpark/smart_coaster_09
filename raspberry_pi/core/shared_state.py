from __future__ import annotations

import copy
import threading
import time
from dataclasses import dataclass, field

from decision.risk_evaluator import RiskState
from fusion.sensor_fusion import WorldState
from planning.escape_planner import EscapePlan
from safety.safety_manager import MotionCommand


@dataclass(frozen=True)
class SystemHealth:
    state: str = "INIT"
    reason: str = "waiting for perception"
    at: float = 0.0  # monotonic clock


@dataclass(frozen=True)
class ControlTiming:
    ticks: int = 0
    last_interval_s: float = 0.0
    last_duration_s: float = 0.0
    max_duration_s: float = 0.0
    missed_deadlines: int = 0


@dataclass(frozen=True)
class SystemSnapshot:
    world: WorldState | None = None
    world_at: float | None = None  # frame acquisition, monotonic clock
    camera_ok: bool = False
    failure: str | None = None
    risk: RiskState | None = None
    plan: EscapePlan | None = None
    command: MotionCommand = field(default_factory=MotionCommand)
    health: SystemHealth = SystemHealth()
    timing: ControlTiming = ControlTiming()
    control_at: float | None = None
    perception_revision: int = 0


class SharedState:
    """Copy at both boundaries so one thread cannot mutate another's snapshot."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = SystemSnapshot()

    def snapshot(self) -> SystemSnapshot:
        with self._lock:
            return copy.deepcopy(self._state)

    def publish_world(self, world: WorldState, *, captured_at: float) -> None:
        with self._lock:
            old = self._state
            self._state = SystemSnapshot(copy.deepcopy(world), captured_at, True, None,
                                         old.risk, old.plan, old.command, old.health,
                                         old.timing, old.control_at, old.perception_revision + 1)

    def publish_failure(self, reason: str, *, camera_ok: bool) -> None:
        with self._lock:
            old = self._state
            self._state = SystemSnapshot(None, None, camera_ok, reason,
                                         old.risk, old.plan, old.command, old.health,
                                         old.timing, old.control_at, old.perception_revision + 1)

    def publish_control(self, *, risk: RiskState, plan: EscapePlan,
                        command: MotionCommand, health: SystemHealth,
                        timing: ControlTiming) -> None:
        with self._lock:
            old = self._state
            self._state = SystemSnapshot(old.world, old.world_at, old.camera_ok, old.failure,
                                         copy.deepcopy(risk), copy.deepcopy(plan),
                                         copy.deepcopy(command), health, timing,
                                         time.monotonic(), old.perception_revision)
