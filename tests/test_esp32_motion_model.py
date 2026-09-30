import math
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "arduino" / "esp32_omni_controller"
HEADER = FIRMWARE / "kinematics_motion.h"
SOURCE = FIRMWARE / "kinematics_motion.cpp"


class State(Enum):
    STOPPED = auto()
    VELOCITY = auto()
    DISTANCE_ACTIVE = auto()
    DISTANCE_BRAKING = auto()
    GOAL_REACHED = auto()
    FAULT = auto()


@dataclass
class Goal:
    motion_id: int
    ux: float
    uy: float
    cruise: float
    distance: float
    progress: float = 0.0
    lateral: float = 0.0
    completed: bool = False


def slew(current, target, accel, decel, dt):
    rate = accel if abs(target) > abs(current) and current * target >= 0 else decel
    step = rate * dt
    return max(current - step, min(current + step, target))


def braking_speed(decel, remaining):
    return math.sqrt(2.0 * decel * max(remaining, 0.0))


def apply_move(state, goal, incoming):
    if goal and incoming.motion_id == goal.motion_id:
        identical = all(
            math.isclose(a, b, abs_tol=1e-3)
            for a, b in zip(
                (incoming.ux, incoming.uy, incoming.distance),
                (goal.ux, goal.uy, goal.distance),
            )
        )
        if not identical:
            return State.FAULT, goal, "MOTION_ID_CHANGED"
        return (State.GOAL_REACHED if goal.completed else state), goal, "REFRESH"
    return State.DISTANCE_ACTIVE, incoming, "START"


def test_body_and_wheel_slew_ramp_through_zero_on_direction_change():
    assert slew(0.0, 10.0, 20.0, 30.0, 0.1) == 2.0
    assert slew(3.0, -10.0, 20.0, 30.0, 0.1) == 0.0
    assert slew(0.0, -10.0, 20.0, 30.0, 0.1) == -2.0


def test_slow_cap_and_wheel_normalization_preserve_ratios():
    vx, vy = 6.0, 8.0
    scale = min(1.0, 5.0 / math.hypot(vx, vy))
    assert (vx * scale, vy * scale) == (3.0, 4.0)
    wheels = [0.0, -25.0, 25.0]
    wheel_scale = 20.0 / max(abs(value) for value in wheels)
    assert [value * wheel_scale for value in wheels] == [0.0, -20.0, 20.0]


def test_braking_bound_enters_braking_and_never_requests_reverse_after_overshoot():
    assert braking_speed(35.0, 10.0) > braking_speed(35.0, 2.0)
    assert braking_speed(35.0, -0.2) == 0.0
    requested = min(10.0, braking_speed(35.0, -0.2))
    assert requested == 0.0


def test_distance_completion_requires_position_speed_lateral_and_valid_encoder():
    def complete(remaining, speed, lateral, encoder_valid):
        return abs(remaining) <= 0.5 and speed <= 0.8 and abs(lateral) <= 2.0 and encoder_valid

    assert complete(0.3, 0.5, 1.0, True)
    assert not complete(0.3, 1.0, 1.0, True)
    assert not complete(0.3, 0.5, 2.1, True)
    assert not complete(0.3, 0.5, 1.0, False)


def test_motion_id_retransmission_refreshes_but_changed_content_faults():
    goal = Goal(7, 1.0, 0.0, 10.0, 30.0)
    state, same_goal, event = apply_move(State.DISTANCE_ACTIVE, goal, Goal(7, 1.0, 0.0, 10.0, 30.0))
    assert (state, same_goal, event) == (State.DISTANCE_ACTIVE, goal, "REFRESH")
    state, _, event = apply_move(State.DISTANCE_ACTIVE, goal, Goal(7, 0.0, 1.0, 10.0, 30.0))
    assert (state, event) == (State.FAULT, "MOTION_ID_CHANGED")
    state, _, event = apply_move(State.DISTANCE_ACTIVE, goal, Goal(7, 1.0, 0.0, 10.0, 31.0))
    assert (state, event) == (State.FAULT, "MOTION_ID_CHANGED")


def test_completed_motion_id_stays_reached_until_a_different_id_arrives():
    goal = Goal(7, 1.0, 0.0, 10.0, 30.0, completed=True)
    state, _, _ = apply_move(State.GOAL_REACHED, goal, Goal(7, 1.0, 0.0, 10.0, 30.0))
    assert state is State.GOAL_REACHED
    state, new_goal, _ = apply_move(State.GOAL_REACHED, goal, Goal(8, 0.0, 1.0, 5.0, 15.0))
    assert state is State.DISTANCE_ACTIVE
    assert new_goal.motion_id == 8


def test_forward_kinematics_keeps_progress_and_lateral_separate():
    d1, d2, d3 = 1.0, -0.5, -0.5
    dx = (d3 - d2) / math.sqrt(3.0)
    dy = (2.0 * d1 - d2 - d3) / 3.0
    assert math.isclose(dx, 0.0, abs_tol=1e-9)
    assert math.isclose(dy, 1.0)
    ux, uy = 1.0, 0.0
    assert dx * ux + dy * uy == 0.0
    assert -dx * uy + dy * ux == 1.0


def test_cpp_motion_contract_has_states_goal_gates_and_immediate_cancel_api():
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    for state in ("STOPPED", "VELOCITY", "DISTANCE_ACTIVE", "DISTANCE_BRAKING", "GOAL_REACHED", "FAULT"):
        assert state in header
    for api in (
        "applyMotionCommand",
        "updateMotionState",
        "computeLimitedWheelTargets",
        "runWheelPid",
        "cancelMotionImmediate",
        "resetMotionAfterFault",
    ):
        assert api in header
    assert "MOTION_ID_CHANGED" in header + source
    assert "brakingSpeedLimit" in source
    assert "PATH_DEVIATION_LIMIT_CM" in source


def test_pid_core_preserves_arrays_gain_order_saturation_and_anti_windup():
    source = SOURCE.read_text(encoding="utf-8")
    for state_array in ("pidIntegral", "pidPreviousError", "measuredWheelSpeed", "targetWheelSpeed"):
        assert state_array in source
    p_index = source.index("const float p =")
    integral_index = source.index("candidateIntegral", p_index)
    derivative_index = source.index("const float d =", integral_index)
    output_index = source.index("float output =", derivative_index)
    anti_windup_index = source.index("pidIntegral[i] = candidateIntegral", output_index)
    saturation_index = source.index("constrain(output", anti_windup_index)
    assert p_index < integral_index < derivative_index < output_index < anti_windup_index < saturation_index
    assert "PID_KP" in source and "PID_KI" in source and "PID_KD" in source
    assert "output = feedForward + p + PID_KI * pidIntegral[i] + d" in source
