import re
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "arduino" / "esp32_omni_controller"
CONFIG = FIRMWARE / "robot_config.h"
SELF_TEST = FIRMWARE / "firmware_self_test.h"


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
