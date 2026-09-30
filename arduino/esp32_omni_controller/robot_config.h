#pragma once

#include <stdint.h>

namespace robot_config {

constexpr uint8_t MOTOR_COUNT = 3;
constexpr uint32_t SERIAL_BAUD = 115200U;

// GitHub smart_coaster_09 main@06693c1 wiring source of truth.
constexpr uint8_t MOTOR_IN1_PINS[MOTOR_COUNT] = {19, 21, 23};
constexpr uint8_t MOTOR_IN2_PINS[MOTOR_COUNT] = {18, 22, 13};
constexpr uint8_t MOTOR_PWM_PINS[MOTOR_COUNT] = {25, 26, 14};
constexpr uint8_t ENCODER_A_PINS[MOTOR_COUNT] = {34, 16, 32};
constexpr uint8_t ENCODER_B_PINS[MOTOR_COUNT] = {35, 17, 33};
constexpr uint8_t MOTOR_STBY_PIN = 27;

// VERIFY WITH PHYSICAL WIRING BEFORE MOTOR TEST.
// GPIO34/35 require external pull-ups. Counts are A-phase RISING (x1).
constexpr float ENCODER_COUNTS_PER_REV = 898.0f;
constexpr bool MOTOR_REVERSED[MOTOR_COUNT] = {false, false, false};
constexpr bool ENCODER_REVERSED[MOTOR_COUNT] = {false, false, false};

constexpr float MATH_PI = 3.14159265358979323846f;
constexpr float WHEEL_RADIUS_CM = 2.9f;
constexpr float ROBOT_RADIUS_CM = 9.0f;
constexpr float WHEEL_ANGLE_DEG[MOTOR_COUNT] = {0.0f, 120.0f, 240.0f};

constexpr float BODY_LINEAR_LIMIT_CM_S = 15.0f;
constexpr float WHEEL_SPEED_LIMIT_CM_S = 20.0f;
constexpr float ANGULAR_LIMIT_RAD_S = 1.0f;
constexpr float SLOW_LINEAR_LIMIT_CM_S = 5.0f;
constexpr float SLOW_ANGULAR_LIMIT_RAD_S = 0.5f;
constexpr float HARD_LINEAR_LIMIT_CM_S = 30.0f;
constexpr float HARD_ANGULAR_LIMIT_RAD_S = 2.0f;
constexpr float MAX_GOAL_DISTANCE_CM = 100.0f;

constexpr float BODY_ACCEL_CM_S2 = 25.0f;
constexpr float BODY_DECEL_CM_S2 = 35.0f;
constexpr float ANGULAR_ACCEL_RAD_S2 = 2.0f;
constexpr float ANGULAR_DECEL_RAD_S2 = 3.0f;
constexpr float WHEEL_ACCEL_CM_S2 = 40.0f;
constexpr float WHEEL_DECEL_CM_S2 = 55.0f;
constexpr float APPROACH_SPEED_CM_S = 1.0f;

constexpr uint32_t CONTROL_PERIOD_US = 10000U;
constexpr uint32_t COMMAND_WATCHDOG_MS = 300U;
constexpr uint32_t MANUAL_WATCHDOG_MS = 3000U;
constexpr uint32_t TELEMETRY_PERIOD_MS = 200U;
constexpr uint32_t WIFI_RETRY_MS = 3000U;
constexpr float MIN_CONTROL_DT_SEC = 0.005f;
constexpr float MAX_CONTROL_DT_SEC = 0.030f;

constexpr uint16_t UDP_COMMAND_PORT = 8888U;
constexpr uint16_t UDP_TELEMETRY_PORT = 8889U;
constexpr uint16_t UDP_PACKET_MAX_BYTES = 512U;
constexpr uint8_t UDP_PACKET_BUDGET = 4U;
constexpr uint32_t UDP_TIME_BUDGET_US = 1000U;

constexpr uint32_t PWM_FREQUENCY_HZ = 20000U;
constexpr uint8_t PWM_RESOLUTION_BITS = 8U;
constexpr int PWM_MAX = 255;
constexpr uint32_t DIRECTION_DEADTIME_US = 100U;
constexpr int PWM_DEADZONE_POS[MOTOR_COUNT] = {0, 0, 0};
constexpr int PWM_DEADZONE_NEG[MOTOR_COUNT] = {0, 0, 0};

// Preserved wheel-speed PID core settings.
constexpr float PID_KP = 3.0f;
constexpr float PID_KI = 0.6f;
constexpr float PID_KD = 0.0f;
constexpr float PID_INTEGRAL_LIMIT = 50.0f;
constexpr bool ENABLE_FEED_FORWARD = true;
constexpr uint8_t FF_POINT_COUNT = 7U;
constexpr float FF_SPEED_CM_S[FF_POINT_COUNT] = {
    0.0f, 2.5f, 5.0f, 7.5f, 10.0f, 12.5f, 15.0f};
constexpr float FF_PWM_POS[MOTOR_COUNT][FF_POINT_COUNT] = {
    {0.0f, 30.0f, 47.0f, 64.0f, 80.0f, 97.0f, 114.0f},
    {0.0f, 27.0f, 46.0f, 61.0f, 75.0f, 98.0f, 120.0f},
    {0.0f, 29.0f, 48.0f, 66.0f, 85.0f, 108.0f, 128.0f}};
constexpr float FF_PWM_NEG[MOTOR_COUNT][FF_POINT_COUNT] = {
    {0.0f, 25.0f, 45.0f, 64.0f, 80.0f, 95.0f, 112.0f},
    {0.0f, 27.0f, 45.0f, 59.0f, 72.0f, 90.0f, 108.0f},
    {0.0f, 29.0f, 48.0f, 69.0f, 88.0f, 125.0f, 155.0f}};
constexpr float SPEED_FILTER_ALPHA = 0.30f;

constexpr float ENCODER_JUMP_SAFETY_MARGIN = 3.0f;
constexpr float DISTANCE_TOLERANCE_CM = 0.5f;
constexpr float LATERAL_TOLERANCE_CM = 2.0f;
constexpr float PATH_DEVIATION_LIMIT_CM = 5.0f;
constexpr float GOAL_STOP_SPEED_CM_S = 0.8f;
constexpr float MOTION_ID_FLOAT_TOLERANCE = 0.001f;
constexpr bool ENABLE_LEGACY_PROTOCOL = true;
constexpr uint32_t LEGACY_SESSION_ID = 0x4C454741U;

}  // namespace robot_config
