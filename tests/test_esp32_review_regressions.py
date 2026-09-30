import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FW = ROOT / "arduino" / "esp32_omni_controller"


def read(name):
    return (FW / name).read_text(encoding="utf-8")


def test_overshoot_inhibits_output_and_manual_pwm_obeys_faults():
    sketch = read("esp32_omni_controller.ino")
    assert "MotionUpdateResult::OUTPUT_INHIBITED" in sketch
    assert "bool manualActuationAllowed()" in sketch
    assert sketch.count("manualActuationAllowed()") >= 3


def test_latched_fault_is_preserved_from_secondary_faults():
    source = read("safety_telemetry.cpp")
    body = source[source.index("void raiseFault"):source.index("bool tryResetFault")]
    assert "safetyState.faultLatched && safetyState.fault != FaultCode::NONE" in body
    assert "preservedCause" in body


def test_shared_acceptance_validates_nonfinite_and_legacy_restart():
    source = read("network_protocol.cpp")
    accept = source[source.index("AcceptanceResult acceptCommand"):]
    assert "validNormalizedCommand(command)" in accept
    assert "isfinite(command.wRadS)" in source
    assert "legacyResyncStop" in accept


def test_same_motion_id_compares_all_immutable_fields_exactly():
    source = read("kinematics_motion.cpp")
    for field in ("goalRequestedSpeedCmS", "goalRequestedWRadS", "goalRequestedStatus"):
        assert field in source
    assert "MOTION_ID_FLOAT_TOLERANCE" not in source


def test_oversized_udp_discard_has_no_unbounded_read_loop():
    sketch = read("esp32_omni_controller.ino")
    body = sketch[sketch.index("void serviceUdpRxBudgeted"):sketch.index("void runControlTickIfDue")]
    oversize = body[body.index("if (packetSize > UDP_PACKET_MAX_BYTES)"):]
    assert "while (commandUdp.available()" not in oversize
    assert "commandUdp.flush()" in oversize


def test_pid_feedforward_gui_and_legacy_telemetry_contracts():
    config = read("robot_config.h")
    assert re.search(r"\bPID_KP\s*=\s*2\.6f\s*;", config)
    assert re.search(r"\bPID_KI\s*=\s*1\.3f\s*;", config)
    motion = read("kinematics_motion.cpp")
    assert "speed > FF_SPEED_CM_S[FF_POINT_COUNT - 1]" in motion
    assert "return 0.0f;" in motion
    gui = (ROOT / "tools" / "laptop_central_controller.py").read_text(encoding="utf-8")
    assert "MAX_LINEAR_CM_S = 15.0" in gui
    assert "MAX_ANGULAR_RAD_S = 1.0" in gui
    telemetry = read("safety_telemetry.cpp")
    for field in ("goal_active", "goal_reached", "target_distance_cm", "action_dtheta_rad"):
        assert f'root["{field}"]' in telemetry
