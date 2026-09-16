#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <math.h>
#include "odometry_math.h"

// =====================================================
// D.I.G / GRISE Wi-Fi Omni Controller
//
// Base hardware mapping: test2.ino
// Added:
//   - Wi-Fi + UDP JSON command receive
//   - {vx, vy, w, status} body velocity commands
//   - 3-wheel omni inverse kinematics
//   - Encoder wheel-speed PID
//   - 300 ms command watchdog
//   - Wi-Fi-loss / invalid-status fail-safe STOP
//   - Existing serial motor-test commands retained
//
// Arduino-ESP32 Core 3.x
// Required library: ArduinoJson v7
//
// ★ status는 "RUN" | "SLOW" | "STOP" 셋만 유효하다 (isValidStatus).
//   그 외 문자열은 invalid로 보고 failClosedStop()이 호출되어
//   강제 정지 + status가 "STOP"으로 덮어써진다.
//   main.py 쪽에서 이 셋 이외의 값을 보내면 안 된다.
// =====================================================

// =====================================================
// 1) USER SETTINGS
// =====================================================

constexpr uint32_t SERIAL_BAUD = 115200;

// ---- Wi-Fi ----
// 반드시 실제 공유기 정보로 바꿔주세요.
const char *WIFI_SSID = "CHANGE_ME";
const char *WIFI_PASS = "CHANGE_ME";

// Raspberry Pi config의 GRISE_ESP32_IP와 맞춰야 한다.
// 실제 공유기 대역과 gateway는 현장에서 확인하세요.
constexpr bool USE_STATIC_IP = true;
IPAddress LOCAL_IP(10, 182, 7, 50);
IPAddress GATEWAY (10, 182, 7, 110);
IPAddress SUBNET(255, 255, 255, 0);
IPAddress DNS1(8, 8, 8, 8);

constexpr uint16_t UDP_PORT = 8888;
constexpr uint16_t TELEMETRY_PORT = 8889;  // NEW: reply to Raspberry Pi
constexpr uint32_t CMD_TIMEOUT_MS = 300;
constexpr uint32_t WIFI_RETRY_MS = 3000;

// ---- PWM ----
constexpr uint32_t PWM_FREQUENCY = 20000;
constexpr uint8_t PWM_RESOLUTION = 8;
constexpr int PWM_MAX = 255;
constexpr int PWM_MIN_MOVE = 45;  // 실제 모터 데드존에 맞춰 튜닝

constexpr uint8_t MOTOR_COUNT = 3;

// ---- Encoder ----
// GA25-370 초기 가정: A phase rising만 센다. 실제 PPR/기어비는 실측 필요.
constexpr float RAW_ENCODER_PPR = 11.0f;
constexpr float GEAR_RATIO = 74.83f;
constexpr float QUADRATURE_MULTIPLIER = 1.0f;
constexpr float COUNTS_PER_OUTPUT_REV = RAW_ENCODER_PPR * GEAR_RATIO * QUADRATURE_MULTIPLIER;
constexpr uint32_t ENCODER_TELEMETRY_MS = 200;

// ---- Robot geometry ----
// 아래 2개 값은 실제 로봇을 자로 재서 최종 보정하세요.
constexpr float WHEEL_RADIUS_CM = 2.9f;
constexpr float ROBOT_RADIUS_CM = 9.0f;

// 휠 장착각: robot +X(front) 기준 CCW.
constexpr float WHEEL_ANGLE_DEG[MOTOR_COUNT] = {0.0f, 120.0f, 240.0f};

// 한 바퀴라도 이 속도를 넘으면 세 바퀴를 같은 비율로 축소.
constexpr float MAX_WHEEL_SPEED_CM_S = 60.0f;

// PC에서 오는 body command의 비정상 값 방어용 상한.
constexpr float MAX_CMD_LINEAR_CM_S = 50.0f;
constexpr float MAX_CMD_ANGULAR_RAD_S = 3.0f;

// status=SLOW일 때 ESP32에서도 한 번 더 상한을 건다.
// PC가 이미 0.4배 감속하므로 여기서는 '배율'이 아니라 최대치만 제한한다.
constexpr float SLOW_MAX_LINEAR_CM_S = 15.0f;
constexpr float SLOW_MAX_ANGULAR_RAD_S = 1.0f;

// ---- Control loop ----
constexpr uint32_t CONTROL_PERIOD_MS = 10; // 100 Hz
constexpr float DISTANCE_TOLERANCE_CM = 0.5f; // initial software value; tune on hardware
constexpr float MAX_GOAL_DISTANCE_CM = 100.0f;
constexpr int64_t MAX_ENCODER_DELTA_PER_TICK = 2000; // reject implausible jumps

// 8-bit PWM용 보수적 초기값. 실제 로봇에서 반드시 튜닝할 것.
float PID_KP = 3.0f;
float PID_KI = 8.0f;
float PID_KD = 0.05f;

// 엔코더 순간 속도 LPF: new = old*(1-a) + raw*a
constexpr float SPEED_FILTER_ALPHA = 0.30f;

// =====================================================
// 2) HARDWARE PIN MAP - test2.ino 그대로 유지
// =====================================================

constexpr uint8_t PIN_STBY = 27;

constexpr uint8_t M1_IN1 = 19;
constexpr uint8_t M1_IN2 = 18;
constexpr uint8_t M1_PWM = 25;
constexpr uint8_t M1_ENC_A = 34;
constexpr uint8_t M1_ENC_B = 35;

constexpr uint8_t M2_IN1 = 21;
constexpr uint8_t M2_IN2 = 22;
constexpr uint8_t M2_PWM = 26;
constexpr uint8_t M2_ENC_A = 16;
constexpr uint8_t M2_ENC_B = 17;

constexpr uint8_t M3_IN1 = 23;
constexpr uint8_t M3_IN2 = 13;
constexpr uint8_t M3_PWM = 14;
constexpr uint8_t M3_ENC_A = 32;
constexpr uint8_t M3_ENC_B = 33;

// =====================================================
// 3) TYPES / GLOBAL STATE
// =====================================================

struct Motor {
    uint8_t in1;
    uint8_t in2;
    uint8_t pwmPin;
    uint8_t encoderA;
    uint8_t encoderB;
    bool motorReversed;
    bool encoderReversed;
    int currentPwm;
};

Motor motors[MOTOR_COUNT] = {
    // motorReversed / encoderReversed는 실측한 wheel 방향에 맞춰 개별 설정한다.
    {M1_IN1, M1_IN2, M1_PWM, M1_ENC_A, M1_ENC_B, false, false, 0},
    {M2_IN1, M2_IN2, M2_PWM, M2_ENC_A, M2_ENC_B, false, false, 0},
    {M3_IN1, M3_IN2, M3_PWM, M3_ENC_A, M3_ENC_B, false, false, 0},
};

enum ControlMode {
    MODE_NETWORK,
    MODE_MANUAL_PWM,
};

ControlMode controlMode = MODE_NETWORK;

WiFiUDP udp;
bool udpStarted = false;
IPAddress controllerIp;
bool controllerIpKnown = false;
uint32_t lastTelemetryUdpMs = 0;
uint32_t lastWifiRetryMs = 0;
char packetBuffer[512];

volatile int32_t encoderCounts[MOTOR_COUNT] = {0, 0, 0};

// 200ms telemetry용
int32_t previousTelemetryCounts[MOTOR_COUNT] = {0, 0, 0};
float encoderRpm[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
uint32_t lastTelemetryMs = 0;
bool streamEnabled = false;

// 100Hz wheel control용
int32_t previousControlCounts[MOTOR_COUNT] = {0, 0, 0};
float targetWheelSpeed[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f}; // cm/s
float measuredWheelSpeed[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f}; // cm/s
float pidIntegral[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
float pidPreviousError[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
uint32_t lastControlMs = 0;
uint32_t controlOverruns = 0; // debug: control intervals above 2 * 10ms

// 최신 네트워크 명령
float cmdVx = 0.0f;  // cm/s, robot forward +
float cmdVy = 0.0f;  // cm/s, robot left +
float cmdW = 0.0f;   // rad/s, CCW +
String cmdStatus = "STOP";
int32_t lastSeq = -1;
uint32_t lastCommandMs = 0;

bool networkStopped = true;
enum GoalMode { IDLE, VELOCITY_CONTROL, DISTANCE_CONTROL, GOAL_REACHED, STOPPED };
GoalMode goalMode = STOPPED;
bool goalArmed = false; // reboot requires an explicit fresh STOP before distance motion
int32_t motionId = -1; // retained after completion/cancel; same ID cannot restart
uint32_t bootId = 0;
float goalTargetCm = 0.0f, goalUx = 0.0f, goalUy = 0.0f;
float goalProgress = 0.0f, goalLateral = 0.0f;
BodyDelta actionDelta = {0.0f, 0.0f, 0.0f};
int32_t goalPreviousCounts[MOTOR_COUNT] = {0, 0, 0};
uint32_t goalStartMs = 0;
uint32_t lastStatusPrintMs = 0;

// =====================================================
// 4) ENCODER ISR
// =====================================================

void IRAM_ATTR encoder1ISR() {
    int direction = digitalRead(M1_ENC_B) ? -1 : 1;
    if (motors[0].encoderReversed) direction = -direction;
    encoderCounts[0] += direction;
}

void IRAM_ATTR encoder2ISR() {
    int direction = digitalRead(M2_ENC_B) ? -1 : 1;
    if (motors[1].encoderReversed) direction = -direction;
    encoderCounts[1] += direction;
}

void IRAM_ATTR encoder3ISR() {
    int direction = digitalRead(M3_ENC_B) ? -1 : 1;
    if (motors[2].encoderReversed) direction = -direction;
    encoderCounts[2] += direction;
}

void snapshotEncoderCounts(int32_t snapshot[MOTOR_COUNT]) {
    noInterrupts();
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) snapshot[i] = encoderCounts[i];
    interrupts();
}

// =====================================================
// 5) MOTOR OUTPUT
// =====================================================

void printMotorName(uint8_t motorIndex) {
    Serial.print("M");
    Serial.print(motorIndex + 1);
}

void writeMotorPwm(uint8_t motorIndex, int pwm, bool verbose = false) {
    if (motorIndex >= MOTOR_COUNT) return;

    Motor &motor = motors[motorIndex];
    pwm = constrain(pwm, -PWM_MAX, PWM_MAX);

    // 방향 전환 전 PWM 제거
    ledcWrite(motor.pwmPin, 0);

    if (pwm == 0) {
        // test2.ino와 같은 Coast 정지
        digitalWrite(motor.in1, LOW);
        digitalWrite(motor.in2, LOW);
        motor.currentPwm = 0;
        if (verbose) {
            printMotorName(motorIndex);
            Serial.println(" stopped.");
        }
        return;
    }

    bool forward = pwm > 0;
    if (motor.motorReversed) forward = !forward;

    if (forward) {
        digitalWrite(motor.in1, HIGH);
        digitalWrite(motor.in2, LOW);
    } else {
        digitalWrite(motor.in1, LOW);
        digitalWrite(motor.in2, HIGH);
    }

    delayMicroseconds(50);

    const int duty = abs(pwm);
    ledcWrite(motor.pwmPin, duty);
    motor.currentPwm = pwm;

    if (verbose) {
        printMotorName(motorIndex);
        Serial.print(" PWM = ");
        Serial.println(pwm);
    }
}

void setAllMotorsManual(int m1, int m2, int m3) {
    controlMode = MODE_MANUAL_PWM;
    goalMode = STOPPED;
    goalArmed = false;
    writeMotorPwm(0, m1, true);
    writeMotorPwm(1, m2, true);
    writeMotorPwm(2, m3, true);
}

void resetPidState() {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        targetWheelSpeed[i] = 0.0f;
        pidIntegral[i] = 0.0f;
        pidPreviousError[i] = 0.0f;
    }
}

void stopAllMotors(bool verbose = false) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        writeMotorPwm(i, 0, false);
    }
    resetPidState();
    if (verbose) Serial.println("All motors stopped.");
}

// =====================================================
// 6) ENCODER SPEED
// =====================================================

void zeroEncoders() {
    if (goalMode == DISTANCE_CONTROL) {
        failClosedStop("encoder reset during goal");
    }
    noInterrupts();
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) encoderCounts[i] = 0;
    interrupts();

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        previousTelemetryCounts[i] = 0;
        previousControlCounts[i] = 0;
        encoderRpm[i] = 0.0f;
        measuredWheelSpeed[i] = 0.0f;
    }

    lastTelemetryMs = millis();
    lastControlMs = millis();
    Serial.println("Encoder counts reset.");
}

void updateWheelSpeedForControl(float dt) {
    int32_t current[MOTOR_COUNT];
    snapshotEncoderCounts(current);

    const float wheelCircumference = 2.0f * (float)M_PI * WHEEL_RADIUS_CM;

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        const int64_t delta = (int64_t)current[i] - previousControlCounts[i];
        if (llabs(delta) > MAX_ENCODER_DELTA_PER_TICK) {
            measuredWheelSpeed[i] = NAN; // updateNetworkControl fails closed
            return;
        }
        previousControlCounts[i] = current[i];

        const float rev = (float)delta / COUNTS_PER_OUTPUT_REV;
        const float rawCmPerSec = (rev * wheelCircumference) / dt;

        measuredWheelSpeed[i] =
            (1.0f - SPEED_FILTER_ALPHA) * measuredWheelSpeed[i] +
            SPEED_FILTER_ALPHA * rawCmPerSec;
    }
}

void updateEncoderTelemetry() {
    const uint32_t now = millis();
    const uint32_t elapsedMs = now - lastTelemetryMs;
    if (elapsedMs < ENCODER_TELEMETRY_MS) return;

    int32_t current[MOTOR_COUNT];
    snapshotEncoderCounts(current);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        const int32_t delta = current[i] - previousTelemetryCounts[i];
        previousTelemetryCounts[i] = current[i];

        encoderRpm[i] =
            ((float)delta * 60000.0f) /
            (COUNTS_PER_OUTPUT_REV * (float)elapsedMs);
    }

    lastTelemetryMs = now;
}

// =====================================================
// 7) OMNI KINEMATICS / PID
// =====================================================

void inverseKinematics(float vx, float vy, float w, float out[MOTOR_COUNT]) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        const float a = WHEEL_ANGLE_DEG[i] * (float)M_PI / 180.0f;
        out[i] = -sinf(a) * vx + cosf(a) * vy + ROBOT_RADIUS_CM * w;
    }

    float maxAbs = 0.0f;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        maxAbs = fmaxf(maxAbs, fabsf(out[i]));
    }

    if (maxAbs > MAX_WHEEL_SPEED_CM_S) {
        const float scale = MAX_WHEEL_SPEED_CM_S / maxAbs;
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) out[i] *= scale;
    }
}

void runWheelPid(float dt) {
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        const float error = targetWheelSpeed[i] - measuredWheelSpeed[i];
        const float p = PID_KP * error;
        const float d = PID_KD * (error - pidPreviousError[i]) / dt;
        pidPreviousError[i] = error;

        const float candidateIntegral = pidIntegral[i] + error * dt;
        float iTerm = PID_KI * candidateIntegral;
        float output = p + iTerm + d;

        // anti-windup
        if (output > -PWM_MAX && output < PWM_MAX) {
            pidIntegral[i] = candidateIntegral;
        } else {
            iTerm = PID_KI * pidIntegral[i];
            output = p + iTerm + d;
        }

        // 정지 목표에서는 잔류 적분 제거
        if (fabsf(targetWheelSpeed[i]) < 0.3f) {
            pidIntegral[i] *= 0.80f;
            if (fabsf(measuredWheelSpeed[i]) < 0.8f) output = 0.0f;
        }

        output = constrain(output, -(float)PWM_MAX, (float)PWM_MAX);
        int pwm = (int)lroundf(output);
        if (pwm != 0 && abs(pwm) < PWM_MIN_MOVE) {
            pwm = pwm > 0 ? PWM_MIN_MOVE : -PWM_MIN_MOVE;
        }
        writeMotorPwm(i, pwm, false);
    }
}

// =====================================================
// 8) UDP COMMAND SAFETY / RECEIVE
// =====================================================

bool isValidStatus(const String &status) {
    return status == "RUN" || status == "SLOW" || status == "STOP";
}

float clampMagnitude(float value, float limit) {
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void applySlowCaps(float &vx, float &vy, float &w) {
    const float linear = hypotf(vx, vy);
    if (linear > SLOW_MAX_LINEAR_CM_S && linear > 1e-6f) {
        const float k = SLOW_MAX_LINEAR_CM_S / linear;
        vx *= k;
        vy *= k;
    }
    w = clampMagnitude(w, SLOW_MAX_ANGULAR_RAD_S);
}

void failClosedStop(const char *reason) {
    goalMode = STOPPED;
    goalArmed = false;
    cmdVx = 0.0f;
    cmdVy = 0.0f;
    cmdW = 0.0f;
    cmdStatus = "STOP";
    stopAllMotors(false);
    networkStopped = true;
    Serial.print("[SAFETY] network command rejected: ");
    Serial.println(reason);
}

bool readFiniteNumber(JsonVariantConst field, float &value) {
    if (!field.is<float>()) return false; // ArduinoJson numeric only; no strings/bools/null
    value = field.as<float>();
    return isfinite(value);
}

bool acceptSequence(int32_t seq, const String &status, uint32_t now) {
    if (lastSeq < 0 || seq > lastSeq) return true;
    // Pi sender restarts seq at 1. After watchdog expiry, require a fresh STOP
    // before adopting that new sequence epoch; old RUN packets never refresh it.
    const bool timedOut = lastCommandMs == 0 || (now - lastCommandMs) > CMD_TIMEOUT_MS;
    return timedOut && status == "STOP";
}

void receiveUdpCommands() {
    if (!udpStarted) return;

    int packetSize = udp.parsePacket();
    while (packetSize > 0) {
        if (packetSize >= (int)sizeof(packetBuffer)) {
            while (udp.available() > 0) udp.read();
            failClosedStop("oversized packet");
        } else {
            const int len = udp.read(packetBuffer, sizeof(packetBuffer) - 1);
            if (len <= 0) {
                failClosedStop("empty packet");
            } else {
                packetBuffer[len] = '\0';

                JsonDocument doc;
                const DeserializationError error = deserializeJson(doc, packetBuffer);

                if (error || !doc.is<JsonObject>()) {
                    failClosedStop("invalid JSON");
                } else {
                    const JsonVariantConst seqField = doc["seq"];
                    const JsonVariantConst statusField = doc["status"];
                    float sentAt = 0.0f, vx = 0.0f, vy = 0.0f, w = 0.0f;
                    if (!seqField.is<int32_t>() || seqField.as<int32_t>() < 0 ||
                        !statusField.is<const char*>() ||
                        !readFiniteNumber(doc["t"], sentAt) || sentAt < 0.0f ||
                        !readFiniteNumber(doc["vx"], vx) ||
                        !readFiniteNumber(doc["vy"], vy) ||
                        !readFiniteNumber(doc["w"], w)) {
                        failClosedStop("missing or invalid command field");
                    } else {
                        String status(statusField.as<const char*>());
                        status.toUpperCase();
                        const int32_t seq = seqField.as<int32_t>();
                        const uint32_t now = millis();
                        if (!isValidStatus(status)) {
                            failClosedStop("invalid status");
                        } else if (hypotf(vx, vy) > MAX_CMD_LINEAR_CM_S ||
                                   fabsf(w) > MAX_CMD_ANGULAR_RAD_S) {
                            failClosedStop("command exceeds speed limit");
                        } else if (!acceptSequence(seq, status, now)) {
                            failClosedStop("duplicate or stale sequence");
                        } else {
                            const bool hasId = !doc["motion_id"].isNull();
                            const bool hasDistance = !doc["target_distance_cm"].isNull();
                            float distance = 0.0f;
                            int32_t incomingId = -1;
                            if (hasId != hasDistance ||
                                (hasId && (!doc["motion_id"].is<int32_t>() ||
                                           doc["motion_id"].as<int32_t>() < 0 ||
                                           !readFiniteNumber(doc["target_distance_cm"], distance) ||
                                           distance <= 0.0f || distance > MAX_GOAL_DISTANCE_CM ||
                                           fabsf(w) > 0.001f || hypotf(vx, vy) < 0.01f))) {
                                failClosedStop("invalid distance goal");
                                packetSize = udp.parsePacket();
                                continue;
                            }
                            if (hasId) incomingId = doc["motion_id"].as<int32_t>();
                            if (status == "SLOW") applySlowCaps(vx, vy, w);
                            if (status == "STOP") vx = vy = w = 0.0f;

                            if (status == "STOP") {
                                goalMode = STOPPED;
                                goalArmed = true;
                            } else if (hasId) {
                                if (controlMode != MODE_NETWORK || !goalArmed ||
                                    (incomingId == motionId &&
                                    goalMode != DISTANCE_CONTROL)) {
                                    vx = vy = w = 0.0f;
                                    if (goalMode != GOAL_REACHED) goalMode = STOPPED;
                                } else if (canStartDistanceGoal(goalArmed,
                                                                 controlMode == MODE_NETWORK,
                                                                 incomingId, motionId)) {
                                    motionId = incomingId;
                                    goalTargetCm = distance;
                                    const float norm = hypotf(vx, vy);
                                    goalUx = vx / norm;
                                    goalUy = vy / norm;
                                    goalProgress = goalLateral = 0.0f;
                                    actionDelta = {0.0f, 0.0f, 0.0f};
                                    snapshotEncoderCounts(goalPreviousCounts);
                                    goalStartMs = now;
                                    goalMode = DISTANCE_CONTROL;
                                } else if (fabsf(distance - goalTargetCm) > 0.001f ||
                                           fabsf(vx / hypotf(vx, vy) - goalUx) > 0.001f ||
                                           fabsf(vy / hypotf(vx, vy) - goalUy) > 0.001f) {
                                    failClosedStop("goal changed under same motion id");
                                    packetSize = udp.parsePacket();
                                    continue;
                                }
                            } else {
                                goalMode = VELOCITY_CONTROL;
                            }

                            cmdVx = vx;
                            cmdVy = vy;
                            cmdW = w;
                            cmdStatus = ((goalMode == STOPPED || goalMode == GOAL_REACHED) &&
                                         status != "STOP") ? "STOP" : status;
                            lastSeq = seq;
                            lastCommandMs = now; // receipt time; Pi epoch t is never compared with millis()
                            controllerIp = udp.remoteIP();
                            controllerIpKnown = true;
                            if (status == "STOP") {
                                stopAllMotors(false);
                                networkStopped = true;
                            }
                        }
                    }
                }
            }
        }

        // backlog가 있으면 가장 최신 packet까지 모두 소비
        packetSize = udp.parsePacket();
    }
}


// =====================================================
// 8-b) UDP TELEMETRY -> RASPBERRY PI
// =====================================================
void sendUdpTelemetry() {
    if (!udpStarted || !controllerIpKnown || WiFi.status() != WL_CONNECTED) return;
    const uint32_t now = millis();
    if ((now - lastTelemetryUdpMs) < ENCODER_TELEMETRY_MS) return;
    lastTelemetryUdpMs = now;

    int32_t counts[MOTOR_COUNT];
    snapshotEncoderCounts(counts);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (!isfinite(encoderRpm[i]) || !isfinite(measuredWheelSpeed[i]) ||
            !isfinite(targetWheelSpeed[i])) {
            failClosedStop("non-finite wheel telemetry");
            encoderRpm[i] = measuredWheelSpeed[i] = targetWheelSpeed[i] = 0.0f;
        }
    }

    JsonDocument doc;
    doc["type"] = "telemetry";
    doc["seq"] = lastSeq;
    doc["ms"] = now;
    bool manualRunning = false;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        manualRunning = manualRunning || motors[i].currentPwm != 0;
    }
    doc["status"] = controlMode == MODE_MANUAL_PWM ?
        (manualRunning ? "RUN" : "STOP") : (networkStopped ? "STOP" : cmdStatus);
    doc["mode"] = controlMode == MODE_NETWORK ? "NETWORK" : "MANUAL_PWM";
    doc["boot_id"] = bootId;
    if (motionId >= 0) doc["motion_id"] = motionId;
    doc["goal_active"] = goalMode == DISTANCE_CONTROL;
    doc["goal_reached"] = goalMode == GOAL_REACHED;
    doc["target_distance_cm"] = goalTargetCm;
    doc["goal_progress_cm"] = goalProgress;
    doc["remaining_distance_cm"] = fmaxf(0.0f, goalTargetCm - goalProgress);
    doc["action_dx_cm"] = actionDelta.dx;
    doc["action_dy_cm"] = actionDelta.dy;
    doc["action_dtheta_rad"] = actionDelta.dtheta;
    doc["lateral_error_cm"] = goalLateral;
    doc["rssi"] = WiFi.RSSI();

    JsonArray c = doc["counts"].to<JsonArray>();
    JsonArray rpm = doc["rpm"].to<JsonArray>();
    JsonArray speed = doc["wheel_speed"].to<JsonArray>();
    JsonArray target = doc["target_speed"].to<JsonArray>();
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        c.add(counts[i]);
        rpm.add(encoderRpm[i]);
        speed.add(measuredWheelSpeed[i]);
        target.add(targetWheelSpeed[i]);
    }

    if (udp.beginPacket(controllerIp, TELEMETRY_PORT)) {
        serializeJson(doc, udp);
        udp.endPacket();
    }
}

// =====================================================
// 9) NETWORK CONTROL LOOP
// =====================================================

void updateNetworkControl() {
    if (controlMode != MODE_NETWORK) return;

    const uint32_t now = millis();
    if ((now - lastControlMs) < CONTROL_PERIOD_MS) return;

    float dt = (now - lastControlMs) / 1000.0f;
    if (dt <= 0.0f) dt = CONTROL_PERIOD_MS / 1000.0f;
    if ((now - lastControlMs) > 2 * CONTROL_PERIOD_MS) controlOverruns++;
    lastControlMs = now;

    updateWheelSpeedForControl(dt);
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (!isfinite(measuredWheelSpeed[i])) {
            failClosedStop("non-finite measured wheel speed");
            measuredWheelSpeed[i] = 0.0f;
            return;
        }
    }

    const bool wifiDown = WiFi.status() != WL_CONNECTED;
    const bool timedOut = (lastCommandMs == 0) || ((now - lastCommandMs) > CMD_TIMEOUT_MS);
    const bool stopStatus = cmdStatus == "STOP";
    const bool shouldStop = wifiDown || timedOut || stopStatus;

    if (shouldStop != networkStopped) {
        networkStopped = shouldStop;
        Serial.print("[NETWORK] ");
        Serial.print(networkStopped ? "STOP" : "RUN");
        Serial.print(" timeout=");
        Serial.print(timedOut);
        Serial.print(" wifiDown=");
        Serial.print(wifiDown);
        Serial.print(" status=");
        Serial.println(cmdStatus);
    }

    if (shouldStop) {
        if (wifiDown || timedOut) { goalMode = STOPPED; goalArmed = false; }
        stopAllMotors(false);
        return;
    }

    if (goalMode == DISTANCE_CONTROL) {
        int32_t current[MOTOR_COUNT];
        snapshotEncoderCounts(current);
        float wheel[MOTOR_COUNT];
        for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
            const int64_t delta = (int64_t)current[i] - goalPreviousCounts[i];
            if (llabs(delta) > MAX_ENCODER_DELTA_PER_TICK) {
                failClosedStop("encoder jump during goal");
                return;
            }
            goalPreviousCounts[i] = current[i];
            wheel[i] = wheelDistanceCm(delta, COUNTS_PER_OUTPUT_REV, WHEEL_RADIUS_CM);
        }
        const BodyDelta step = forwardKinematics(wheel[0], wheel[1], wheel[2], ROBOT_RADIUS_CM);
        const float midHeading = actionDelta.dtheta + step.dtheta * 0.5f;
        actionDelta.dx += cosf(midHeading) * step.dx - sinf(midHeading) * step.dy;
        actionDelta.dy += sinf(midHeading) * step.dx + cosf(midHeading) * step.dy;
        actionDelta.dtheta += step.dtheta;
        goalProgress = goalProgressCm(actionDelta, goalUx, goalUy);
        goalLateral = lateralErrorCm(actionDelta, goalUx, goalUy);
        if (!isfinite(goalProgress) || !isfinite(goalLateral) ||
            !isfinite(actionDelta.dtheta)) {
            failClosedStop("invalid odometry");
            return;
        }
        if (distanceGoalReached(goalProgress, goalTargetCm, DISTANCE_TOLERANCE_CM)) {
            goalMode = GOAL_REACHED;
            cmdVx = cmdVy = cmdW = 0.0f;
            cmdStatus = "STOP";
            networkStopped = true;
            stopAllMotors(false);
            return;
        }
    }

    float wheelTargets[MOTOR_COUNT];
    inverseKinematics(cmdVx, cmdVy, cmdW, wheelTargets);
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        targetWheelSpeed[i] = wheelTargets[i];
    }

    runWheelPid(dt);
}

// =====================================================
// 10) WI-FI
// =====================================================

void startUdpIfNeeded() {
    if (WiFi.status() == WL_CONNECTED && !udpStarted) {
        if (udp.begin(UDP_PORT)) {
            udpStarted = true;
            Serial.print("[UDP] listening on port ");
            Serial.println(UDP_PORT);
        }
    }
}

void beginWiFi() {
    WiFi.mode(WIFI_STA);

    if (USE_STATIC_IP) {
        if (!WiFi.config(LOCAL_IP, GATEWAY, SUBNET, DNS1)) {
            Serial.println("[WiFi] static IP configuration failed");
        }
    }

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    lastWifiRetryMs = millis();
    Serial.print("[WiFi] connecting to ");
    Serial.println(WIFI_SSID);
}

void maintainWiFi() {
    if (WiFi.status() == WL_CONNECTED) {
        startUdpIfNeeded();
        return;
    }

    if (udpStarted) {
        udp.stop();
        udpStarted = false;
    }

    // A reconnect must not resume the previous command or manual PWM output.
    if (!networkStopped || controlMode == MODE_MANUAL_PWM) stopAllMotors(false);
    cmdVx = cmdVy = cmdW = 0.0f;
    cmdStatus = "STOP";
    networkStopped = true;
    goalMode = STOPPED;
    goalArmed = false;
    lastCommandMs = 0;
    lastSeq = -1;
    controllerIpKnown = false;

    const uint32_t now = millis();
    if ((now - lastWifiRetryMs) >= WIFI_RETRY_MS) {
        lastWifiRetryMs = now;
        Serial.println("[WiFi] reconnecting...");
        WiFi.disconnect();
        WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
}

// =====================================================
// 11) SERIAL DEBUG / MANUAL COMMANDS
// =====================================================

void printEncoderStatus() {
    int32_t counts[MOTOR_COUNT];
    snapshotEncoderCounts(counts);

    Serial.println();
    Serial.println("===== ENCODER =====");
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        printMotorName(i);
        Serial.print(" Count=");
        Serial.print(counts[i]);
        Serial.print(" RPM=");
        Serial.print(encoderRpm[i], 2);
        Serial.print(" speed=");
        Serial.print(measuredWheelSpeed[i], 2);
        Serial.println(" cm/s");
    }
    Serial.print("COUNTS/REV=");
    Serial.println(COUNTS_PER_OUTPUT_REV, 2);
    Serial.println("===================");
}

void printStatus() {
    Serial.println();
    Serial.println("===== STATUS =====");
    Serial.print("MODE = ");
    Serial.println(controlMode == MODE_NETWORK ? "NETWORK" : "MANUAL_PWM");

    Serial.print("WiFi = ");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.print("CONNECTED  IP=");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("DISCONNECTED");
    }

    Serial.print("UDP = ");
    Serial.println(udpStarted ? "ON" : "OFF");

    Serial.print("cmd = (");
    Serial.print(cmdVx, 1);
    Serial.print(", ");
    Serial.print(cmdVy, 1);
    Serial.print(", ");
    Serial.print(cmdW, 2);
    Serial.print(") status=");
    Serial.print(cmdStatus);
    Serial.print(" seq=");
    Serial.println(lastSeq);

    const uint32_t age = lastCommandMs == 0 ? 0xFFFFFFFFUL : millis() - lastCommandMs;
    Serial.print("command age ms = ");
    if (lastCommandMs == 0) Serial.println("NONE");
    else Serial.println(age);
    Serial.print("control overruns (>20ms) = ");
    Serial.println(controlOverruns);

    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        printMotorName(i);
        Serial.print(" target=");
        Serial.print(targetWheelSpeed[i], 2);
        Serial.print(" cm/s measured=");
        Serial.print(measuredWheelSpeed[i], 2);
        Serial.print(" pwm=");
        Serial.println(motors[i].currentPwm);
    }

    Serial.println("==================");
}

void printPinMap() {
    Serial.println();
    Serial.println("===== PIN MAP =====");
    Serial.println("STBY=27");
    Serial.println("M1 IN1=19 IN2=18 PWM=25 ENC_A=34 ENC_B=35");
    Serial.println("M2 IN1=21 IN2=22 PWM=26 ENC_A=16 ENC_B=17");
    Serial.println("M3 IN1=23 IN2=13 PWM=14 ENC_A=32 ENC_B=33");
    Serial.println("===================");
}

void printHelp() {
    Serial.println();
    Serial.println("===== COMMANDS =====");
    Serial.println("AUTO                -> network control mode");
    Serial.println("M1 100              -> manual PWM mode");
    Serial.println("M2 -100");
    Serial.println("M3 100");
    Serial.println("ALL 100 100 100     -> manual PWM mode");
    Serial.println("STOP                -> manual STOP; use AUTO to resume network");
    Serial.println("ENC");
    Serial.println("ZERO");
    Serial.println("STREAM ON");
    Serial.println("STREAM OFF");
    Serial.println("STATUS");
    Serial.println("PIN");
    Serial.println("HELP");
    Serial.println("====================");
}

void handleSingleMotorCommand(const String &command) {
    int motorNumber = 0;
    int pwm = 0;
    if (sscanf(command.c_str(), "M%d %d", &motorNumber, &pwm) != 2) {
        Serial.println("ERROR: use M1 100");
        return;
    }
    if (motorNumber < 1 || motorNumber > 3) {
        Serial.println("ERROR: motor must be 1~3");
        return;
    }

    if (controlMode != MODE_MANUAL_PWM) {
        stopAllMotors(false);
    }
    controlMode = MODE_MANUAL_PWM;
    goalMode = STOPPED;
    goalArmed = false;
    resetPidState();
    writeMotorPwm((uint8_t)(motorNumber - 1), pwm, true);
    Serial.println("[MODE] MANUAL_PWM");
}

void handleAllMotorCommand(const String &command) {
    int m1 = 0, m2 = 0, m3 = 0;
    if (sscanf(command.c_str(), "ALL %d %d %d", &m1, &m2, &m3) != 3) {
        Serial.println("ERROR: use ALL 100 100 100");
        return;
    }
    setAllMotorsManual(m1, m2, m3);
    resetPidState();
    Serial.println("[MODE] MANUAL_PWM");
}

void enterNetworkMode() {
    stopAllMotors(false);
    controlMode = MODE_NETWORK;
    goalMode = STOPPED;
    goalArmed = false;
    cmdVx = cmdVy = cmdW = 0.0f;
    cmdStatus = "STOP";
    lastCommandMs = 0; // 반드시 새 packet을 받아야 재주행
    networkStopped = true;

    int32_t current[MOTOR_COUNT];
    snapshotEncoderCounts(current);
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        previousControlCounts[i] = current[i];
        pidIntegral[i] = 0.0f;
        pidPreviousError[i] = 0.0f;
    }
    lastControlMs = millis();

    Serial.println("[MODE] NETWORK - waiting for a fresh UDP command");
}

void handleSerialCommand() {
    if (Serial.available() <= 0) return;

    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    if (command.length() == 0) return;

    if (command.startsWith("M1 ") || command.startsWith("M2 ") || command.startsWith("M3 ")) {
        handleSingleMotorCommand(command);
    } else if (command.startsWith("ALL ")) {
        handleAllMotorCommand(command);
    } else if (command == "STOP") {
        controlMode = MODE_MANUAL_PWM;
        goalMode = STOPPED;
        goalArmed = false;
        stopAllMotors(true);
        Serial.println("[MODE] MANUAL_PWM. Send AUTO to resume network control.");
    } else if (command == "AUTO") {
        enterNetworkMode();
    } else if (command == "ENC") {
        printEncoderStatus();
    } else if (command == "ZERO") {
        zeroEncoders();
    } else if (command == "STREAM ON") {
        streamEnabled = true;
        Serial.println("Encoder stream ON");
    } else if (command == "STREAM OFF") {
        streamEnabled = false;
        Serial.println("Encoder stream OFF");
    } else if (command == "STATUS") {
        printStatus();
    } else if (command == "PIN") {
        printPinMap();
    } else if (command == "HELP") {
        printHelp();
    } else {
        Serial.println("Unknown command. Send HELP.");
    }
}

// =====================================================
// 12) INITIALIZATION
// =====================================================

bool initializeMotor(Motor &motor) {
    pinMode(motor.in1, OUTPUT);
    pinMode(motor.in2, OUTPUT);
    digitalWrite(motor.in1, LOW);
    digitalWrite(motor.in2, LOW);

    const bool attached = ledcAttach(motor.pwmPin, PWM_FREQUENCY, PWM_RESOLUTION);
    if (!attached) return false;

    ledcWrite(motor.pwmPin, 0);
    motor.currentPwm = 0;
    return true;
}

void initializeEncoders() {
    // GPIO34/35: internal pull-up 없음
    pinMode(M1_ENC_A, INPUT);
    pinMode(M1_ENC_B, INPUT);

    pinMode(M2_ENC_A, INPUT_PULLUP);
    pinMode(M2_ENC_B, INPUT_PULLUP);
    pinMode(M3_ENC_A, INPUT_PULLUP);
    pinMode(M3_ENC_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(M1_ENC_A), encoder1ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(M2_ENC_A), encoder2ISR, RISING);
    attachInterrupt(digitalPinToInterrupt(M3_ENC_A), encoder3ISR, RISING);

    zeroEncoders();
}

void setup() {
    bootId = esp_random();
    Serial.begin(SERIAL_BAUD);
    Serial.setTimeout(30);
    delay(400);

    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, LOW);

    bool ok = true;
    for (uint8_t i = 0; i < MOTOR_COUNT; i++) {
        if (!initializeMotor(motors[i])) {
            Serial.print("ERROR: M");
            Serial.print(i + 1);
            Serial.println(" PWM initialization failed");
            ok = false;
        }
    }

    if (!ok) {
        digitalWrite(PIN_STBY, LOW);
        Serial.println("Motor driver disabled due to PWM init failure");
        while (true) delay(1000);
    }

    initializeEncoders();

    // 모든 PWM=0 확인 후 TB6612 활성화
    digitalWrite(PIN_STBY, HIGH);
    stopAllMotors(false);

    beginWiFi();

    Serial.println();
    Serial.println("============================================");
    Serial.println("D.I.G / GRISE Wi-Fi omni controller ready");
    Serial.println("Default mode: NETWORK");
    Serial.println("UDP JSON: {seq,vx,vy,w,status}");
    Serial.println("Port: 8888 / watchdog: 300 ms");
    Serial.println("Send HELP for serial debug commands");
    Serial.println("============================================");
}

// =====================================================
// 13) MAIN LOOP
// =====================================================

void loop() {
    handleSerialCommand();
    maintainWiFi();
    receiveUdpCommands();

    updateNetworkControl();
    updateEncoderTelemetry();
    sendUdpTelemetry();

    if (streamEnabled && (millis() - lastStatusPrintMs) >= 1000) {
        lastStatusPrintMs = millis();
        printStatus();
    }
}
