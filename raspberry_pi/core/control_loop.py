from __future__ import annotations

import time

import config
from core.shared_state import ControlTiming, SharedState, SystemHealth
from core.state_machine import ControlStateMachine
from core.motion_goal import MotionGoals
from decision import RiskEvaluator
from decision.risk_evaluator import RiskState
from planning.escape_planner import EscapePlan, EscapePlanner
from safety import SafetyManager
from safety.safety_manager import MotionCommand


class ControlLoop:
    def __init__(self, shared: SharedState, telemetry_rx, sender) -> None:
        self.shared = shared
        self.telemetry_rx = telemetry_rx
        self.sender = sender
        self.machine = ControlStateMachine()
        self.risk_eval = RiskEvaluator()
        self.planner = EscapePlanner()
        self.safety = SafetyManager()
        self.timing = ControlTiming()
        self._last_tick: float | None = None
        self.motion_goals = MotionGoals()

    def tick(self, *, now: float | None = None) -> MotionCommand:
        start = time.monotonic() if now is None else now
        interval = 0.0 if self._last_tick is None else start - self._last_tick
        self._last_tick = start
        risk = RiskState()
        plan = EscapePlan()
        state = None
        try:
            state = self.shared.snapshot()
            health = self.machine.assess(state, now=start)
            if health.state != "READY":
                command = MotionCommand(reason=health.reason)
                risk = self.risk_eval.invalidate(health.reason, start)
            else:
                world = state.world
                risk = self.risk_eval.evaluate(world if world.vision is not None else world.radar_target)
                plan = self.planner.plan(world, risk)
                command = self.safety.validate(
                    camera_ok=state.camera_ok, world=world, plan=plan,
                    telemetry=self.telemetry_rx.latest(),
                    risk=risk,
                )
                latest = self.shared.snapshot()
                if latest.perception_revision != state.perception_revision:
                    health = SystemHealth("STOP", "WorldState changed during control", start)
                    command = MotionCommand(reason=health.reason)
                    risk = self.risk_eval.invalidate(health.reason, start)
                else:
                    final_health = self.machine.assess(
                        latest, now=time.monotonic() if now is None else start
                    )
                    if final_health.state != "READY":
                        health = final_health
                        command = MotionCommand(reason=health.reason)
                        risk = self.risk_eval.invalidate(health.reason, start)
        except Exception as exc:
            health = SystemHealth("FAULT", f"control exception: {exc}", start)
            command = MotionCommand(reason=health.reason)
            risk = self.risk_eval.invalidate(health.reason, start)
            print(f"[CONTROL] {health.reason}")

        # The scheduled loop owns its cadence; do not let the sender's throttle skip a tick.
        if (state is not None and state.world is not None and state.world.vision is not None and
                command.status in ("RUN", "SLOW")):
            goal = self.motion_goals.select(command, plan, self.telemetry_rx.latest(),
                                            state.perception_revision)
            if goal is None:
                command = MotionCommand(reason="distance goal synchronization or replan")
                self.sender.send(0, 0, 0, "STOP", force=True)
            else:
                self.sender.send(command.vx, command.vy, command.w, command.status,
                                 force=True, motion_id=goal[0], target_distance_cm=goal[1])
        else:
            self.motion_goals.select(MotionCommand(), plan, None, 0)
            self.sender.send(command.vx, command.vy, command.w, command.status, force=True)
        duration = max(0.0, time.monotonic() - start) if now is None else 0.0
        self.timing = ControlTiming(
            ticks=self.timing.ticks + 1,
            last_interval_s=interval,
            last_duration_s=duration,
            max_duration_s=max(self.timing.max_duration_s, duration),
            missed_deadlines=self.timing.missed_deadlines + (duration > 1 / config.UDP_SEND_HZ),
        )
        self.shared.publish_control(risk=risk, plan=plan, command=command,
                                    health=health, timing=self.timing)
        return command

    def run(self, stop_event) -> None:
        period = 1.0 / config.UDP_SEND_HZ
        deadline = time.monotonic()
        while not stop_event.is_set():
            self.tick()
            deadline += period
            now = time.monotonic()
            if deadline <= now:
                deadline = now + period
            stop_event.wait(max(0.0, deadline - now))


def run_monitor_loop(shared: SharedState, telemetry_rx, monitor, stop_event) -> None:
    period = 1.0 / config.MONITOR_SEND_HZ
    while not stop_event.is_set():
        state = shared.snapshot()
        try:
            if state.world is None or state.risk is None or state.plan is None:
                monitor.send_unavailable(state.failure or state.health.reason,
                                         camera_ok=state.camera_ok)
            else:
                monitor.send(world=state.world, risk=state.risk, plan=state.plan,
                             command=state.command, telemetry=telemetry_rx.latest(),
                             camera_ok=state.camera_ok)
        except Exception as exc:
            print(f"[MONITOR] update failed: {exc}")
        stop_event.wait(period)
