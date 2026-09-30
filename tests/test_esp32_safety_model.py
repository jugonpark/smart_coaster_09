from enum import Enum, auto
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "arduino" / "esp32_omni_controller"
HEADER = FIRMWARE / "safety_telemetry.h"
SOURCE = FIRMWARE / "safety_telemetry.cpp"


class Fault(Enum):
    NONE = auto()
    WIFI_LOSS = auto()
    CMD_TIMEOUT = auto()
    BAD_PACKET = auto()
    SPEED_LIMIT = auto()
    ENCODER_JUMP = auto()
    ENCODER_INVALID = auto()
    ODOMETRY_INVALID = auto()
    PATH_DEVIATION = auto()
    CONTROL_TIMING = auto()


RECOVERABLE = {Fault.WIFI_LOSS, Fault.CMD_TIMEOUT, Fault.BAD_PACKET}


def elapsed_u32(now, then):
    return (now - then) & 0xFFFFFFFF


def watchdog_fault(mode, moving, now, last_network, last_manual, wifi):
    if not moving:
        return Fault.NONE
    if mode == "NETWORK":
        if not wifi:
            return Fault.WIFI_LOSS
        if elapsed_u32(now, last_network) > 300:
            return Fault.CMD_TIMEOUT
    elif elapsed_u32(now, last_manual) > 3000:
        return Fault.CMD_TIMEOUT
    return Fault.NONE


def can_clear(fault, condition_clear, explicit_reset, handshake):
    if fault is Fault.NONE or not condition_clear or not handshake:
        return False
    return fault in RECOVERABLE or explicit_reset


def test_network_watchdog_covers_velocity_and_distance_and_wifi_loss():
    for moving_state in ("VELOCITY", "DISTANCE_ACTIVE", "DISTANCE_BRAKING"):
        assert watchdog_fault("NETWORK", moving_state, 1301, 1000, 0, True) is Fault.CMD_TIMEOUT
        assert watchdog_fault("NETWORK", moving_state, 1050, 1000, 0, False) is Fault.WIFI_LOSS


def test_manual_watchdog_is_independent_of_wifi_and_uses_three_seconds():
    assert watchdog_fault("MANUAL_TEST", True, 3999, 0, 1000, False) is Fault.NONE
    assert watchdog_fault("MANUAL_TEST", True, 4001, 0, 1000, False) is Fault.CMD_TIMEOUT


def test_watchdog_age_is_correct_across_uint32_wraparound():
    then = 0xFFFFFFF0
    assert elapsed_u32(0x20, then) == 48
    assert watchdog_fault("NETWORK", True, 0x20, then, 0, True) is Fault.NONE
    assert watchdog_fault("NETWORK", True, 0x200, then, 0, True) is Fault.CMD_TIMEOUT


def test_recoverable_fault_requires_clear_condition_and_fresh_handshake():
    assert not can_clear(Fault.BAD_PACKET, True, False, False)
    assert can_clear(Fault.BAD_PACKET, True, False, True)
    assert not can_clear(Fault.BAD_PACKET, False, False, True)


def test_latched_fault_requires_explicit_reset_and_handshake():
    for fault in (Fault.SPEED_LIMIT, Fault.ENCODER_JUMP, Fault.ENCODER_INVALID,
                  Fault.ODOMETRY_INVALID, Fault.PATH_DEVIATION, Fault.CONTROL_TIMING):
        assert not can_clear(fault, True, False, True)
        assert not can_clear(fault, True, True, False)
        assert can_clear(fault, True, True, True)


def test_cpp_immediate_stop_writes_motor_zero_before_state_cleanup():
    source = SOURCE.read_text(encoding="utf-8")
    start = source.index("void immediateStopNow")
    end = source.index("void raiseFault", start)
    body = source[start:end]
    assert body.index("stopAllMotorsImmediate") < body.index("clearMotionMailbox")
    assert body.index("stopAllMotorsImmediate") < body.index("cancelMotionImmediate")


def test_cpp_fault_inventory_watchdogs_and_recovery_match_contract():
    header = HEADER.read_text(encoding="utf-8")
    source = SOURCE.read_text(encoding="utf-8")
    for fault in Fault.__members__:
        assert fault in header
    for api in ("immediateStopNow", "raiseFault", "tryResetFault", "updateWatchdogs", "buildTelemetry", "sendTelemetry"):
        assert api in header
    assert "COMMAND_WATCHDOG_MS" in source
    assert "MANUAL_WATCHDOG_MS" in source
    assert "uint32_t(nowMs -" in source


def test_telemetry_contains_typed_fields_and_legacy_aliases_without_fake_goal_values():
    source = SOURCE.read_text(encoding="utf-8")
    typed = (
        "type", "boot_id", "session_id", "last_seq", "uptime_ms", "mode", "state",
        "fault", "cmd_vx", "cmd_vy", "cmd_w", "command_age_ms", "wheel_target",
        "wheel_speed", "wheel_pwm", "encoder_count", "motion_id", "goal_target_cm",
        "goal_progress_cm", "remaining_cm", "lateral_error_cm", "overshoot_cm",
        "odom_dx_cm", "odom_dy_cm", "odom_dtheta_rad", "wifi_rssi", "control_overruns",
    )
    aliases = ("seq", "status", "counts", "rpm", "target_speed", "pwm")
    for field in typed + aliases:
        assert f'"{field}"' in source
    assert "nullptr" in source
