import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "arduino" / "esp32_omni_controller"
CONFIG = FIRMWARE / "robot_config.h"
SELF_TEST = FIRMWARE / "firmware_self_test.h"
MOTOR_HEADER = FIRMWARE / "motor_encoder.h"
MOTOR_SOURCE = FIRMWARE / "motor_encoder.cpp"


def _config_text() -> str:
    return CONFIG.read_text(encoding="utf-8")


def _int_array(source: str, name: str) -> list[int]:
    match = re.search(rf"\b{name}\s*\[[^]]*\]\s*=\s*\{{([^}}]+)\}}", source)
    assert match, f"missing array {name}"
    return [int(value.strip()) for value in match.group(1).split(",")]


def test_pin_map_matches_verified_github_wiring_without_output_conflicts():
    source = _config_text()
    assert _int_array(source, "MOTOR_IN1_PINS") == [19, 21, 23]
    assert _int_array(source, "MOTOR_IN2_PINS") == [18, 22, 13]
    assert _int_array(source, "MOTOR_PWM_PINS") == [25, 26, 14]
    assert _int_array(source, "ENCODER_A_PINS") == [34, 16, 32]
    assert _int_array(source, "ENCODER_B_PINS") == [35, 17, 33]
    assert re.search(r"\bMOTOR_STBY_PIN\s*=\s*27\b", source)

    motor_outputs = (
        _int_array(source, "MOTOR_IN1_PINS")
        + _int_array(source, "MOTOR_IN2_PINS")
        + _int_array(source, "MOTOR_PWM_PINS")
        + [27]
    )
    assert len(motor_outputs) == len(set(motor_outputs))


def test_controller_limits_and_periods_have_expected_units_and_values():
    source = _config_text()
    expected = {
        "BODY_LINEAR_LIMIT_CM_S": "15.0f",
        "WHEEL_SPEED_LIMIT_CM_S": "20.0f",
        "ANGULAR_LIMIT_RAD_S": "1.0f",
        "SLOW_LINEAR_LIMIT_CM_S": "5.0f",
        "CONTROL_PERIOD_US": "10000U",
        "COMMAND_WATCHDOG_MS": "300U",
        "TELEMETRY_PERIOD_MS": "200U",
        "UDP_COMMAND_PORT": "8888U",
        "UDP_TELEMETRY_PORT": "8889U",
    }
    for name, value in expected.items():
        assert re.search(rf"\b{name}\s*=\s*{re.escape(value)}\s*;", source), name


def test_encoder_configuration_carries_physical_verification_warning():
    source = _config_text()
    assert "VERIFY WITH PHYSICAL WIRING BEFORE MOTOR TEST" in source
    assert re.search(r"\bENCODER_COUNTS_PER_REV\s*=\s*89[5-9](?:\.\d+)?f\s*;", source)
    assert "MAX_ENCODER_DELTA_PER_TICK = 2000" not in source


def test_secrets_are_local_and_template_contains_no_credentials():
    ignore_text = (ROOT / ".gitignore").read_text(encoding="utf-8")
    template = (FIRMWARE / "secrets.example.h").read_text(encoding="utf-8")
    assert "arduino/esp32_omni_controller/secrets.h" in ignore_text
    assert "#define WIFI_SSID" in template
    assert "#define WIFI_PASSWORD" in template
    assert "YOUR_WIFI_SSID" in template
    assert "YOUR_WIFI_PASSWORD" in template


def test_compile_time_firmware_self_tests_cover_math_contract():
    source = SELF_TEST.read_text(encoding="utf-8")
    assert source.count("static_assert") >= 10
    for behavior in (
        "inverseKinematicsBody",
        "forwardKinematics",
        "goalProgressCm",
        "lateralErrorCm",
        "encoderDeltaLimitCounts",
        "brakingSpeedLimit",
        "slewTowards",
    ):
        assert behavior in source


def test_motor_initialization_keeps_driver_disabled_until_outputs_are_zero():
    source = MOTOR_SOURCE.read_text(encoding="utf-8")
    init_start = source.index("void initializeMotorOutputsSafe")
    enable_start = source.index("void enableMotorDriver")
    init_body = source[init_start:enable_start]
    assert init_body.index("digitalWrite(MOTOR_STBY_PIN, LOW)") < init_body.index("ledcAttach")
    assert init_body.index("ledcAttach") < init_body.rindex("ledcWrite")
    assert "digitalWrite(MOTOR_STBY_PIN, HIGH)" not in init_body
    assert "digitalWrite(MOTOR_STBY_PIN, HIGH)" in source[enable_start:]


def test_motor_direction_change_zeros_pwm_before_deadtime_and_pin_change():
    source = MOTOR_SOURCE.read_text(encoding="utf-8")
    start = source.index("void writeMotorPwm")
    end = source.index("void stopAllMotorsImmediate", start)
    body = source[start:end]
    assert body.index("ledcWrite") < body.index("delayMicroseconds")
    assert body.index("delayMicroseconds") < body.index("digitalWrite")
    assert "constrain" in body


def test_encoder_isrs_only_read_direction_and_update_one_count():
    source = MOTOR_SOURCE.read_text(encoding="utf-8")
    for name in ("onEncoderA0", "onEncoderA1", "onEncoderA2"):
        start = source.index(f"void IRAM_ATTR {name}")
        body = source[start : source.index("}", start) + 1]
        assert "digitalRead" in body
        assert "encoderCounts" in body
        assert "Serial" not in body
        assert "float" not in body
    assert "attachInterrupt" in source
    assert "RISING" in source


def test_encoder_snapshot_is_atomic_and_speed_filter_rejects_derived_jumps():
    source = MOTOR_SOURCE.read_text(encoding="utf-8")
    assert "noInterrupts()" in source
    assert "interrupts()" in source
    assert "encoderDeltaLimitCounts" in source
    assert "SPEED_FILTER_ALPHA" in source
    header = MOTOR_HEADER.read_text(encoding="utf-8")
    for api in (
        "initializeMotorOutputsSafe",
        "enableMotorDriver",
        "disableMotorDriver",
        "writeMotorPwm",
        "stopAllMotorsImmediate",
        "attachEncoderInterrupts",
        "snapshotEncoderCounts",
        "updateMeasuredWheelSpeed",
        "zeroEncoderReference",
    ):
        assert api in header


def test_math_headers_compile_alongside_arduino_and_legacy_sketch_during_migration():
    config = _config_text()
    math_header = (FIRMWARE / "odometry_math.h").read_text(encoding="utf-8")
    assert not re.search(r"constexpr\s+float\s+PI\b", config)
    assert "canStartDistanceGoal" not in math_header
    assert "distanceGoalReached" not in math_header


def test_encoder_snapshot_uses_single_strong_module_implementation():
    source = MOTOR_SOURCE.read_text(encoding="utf-8")
    assert "__attribute__((weak))" not in source


def test_sketch_setup_and_loop_preserve_safe_bounded_service_order():
    source = (FIRMWARE / "esp32_omni_controller.ino").read_text(encoding="utf-8")
    setup_start = source.index("void setup()")
    loop_start = source.index("void loop()")
    setup = source[setup_start:loop_start]
    loop = source[loop_start:]
    ordered_setup = (
        "Serial.begin",
        "initializeMotorOutputsSafe",
        "attachEncoderInterrupts",
        "resetMotionAfterFault",
        "beginWiFi",
        "enableMotorDriver",
    )
    positions = [setup.index(token) for token in ordered_setup]
    assert positions == sorted(positions)
    first_control = loop.index("runControlTickIfDue")
    udp = loop.index("serviceUdpRxBudgeted")
    second_control = loop.index("runControlTickIfDue", first_control + 1)
    telemetry = loop.index("runTelemetryIfDue")
    assert first_control < udp < second_control < telemetry


def test_udp_service_enforces_packet_and_time_budgets():
    source = (FIRMWARE / "esp32_omni_controller.ino").read_text(encoding="utf-8")
    start = source.index("void serviceUdpRxBudgeted")
    end = source.index("void runControlTickIfDue", start)
    body = source[start:end]
    assert "UDP_PACKET_BUDGET" in body
    assert "UDP_TIME_BUDGET_US" in body
    assert "micros()" in body
    assert "packetsHandled" in body
    assert "decodePacket" in body
    assert "handleAcceptedCommand(decoded.command" in body
    acceptance_start = source.index("void handleAcceptedCommand")
    acceptance_end = source.index("NormalizedCommand manualCommand", acceptance_start)
    assert "acceptCommand" in source[acceptance_start:acceptance_end]


def test_control_path_has_no_json_or_serial_output_and_manual_commands_are_complete():
    source = (FIRMWARE / "esp32_omni_controller.ino").read_text(encoding="utf-8")
    start = source.index("void runControlTickIfDue")
    end = source.index("void runTelemetryIfDue", start)
    body = source[start:end]
    assert "Serial." not in body
    assert "Json" not in body
    for command in ("HELP", "STATUS", "PIN", "ENC", "ZERO", "AUTO", "MANUAL", "STOP", "M1", "M2", "M3", "ALL", "VEL", "MOVE"):
        assert f'"{command}"' in source


def test_sketch_uses_local_secrets_header_without_tracked_credentials():
    source = (FIRMWARE / "esp32_omni_controller.ino").read_text(encoding="utf-8")
    assert '#include "secrets.h"' in source
    assert "__has_include" in source
    assert not re.search(r'const\s+char\s*\*\s*WIFI_(?:SSID|PASS)\s*=\s*"(?!YOUR_)', source)
