#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <esp_system.h>
#include <math.h>
#include <string.h>

#include "firmware_self_test.h"
#include "kinematics_motion.h"
#include "motor_encoder.h"
#include "network_protocol.h"
#include "odometry_math.h"
#include "robot_config.h"
#include "safety_telemetry.h"

#if __has_include("secrets.h")
#include "secrets.h"
#else
#define WIFI_SSID "YOUR_WIFI_SSID"
#define WIFI_PASSWORD "YOUR_WIFI_PASSWORD"
#endif

using namespace robot_config;
using namespace motion_control;

namespace {

WiFiUDP commandUdp;
CommandMailbox commandMailbox;
CommandAcceptanceState commandAcceptance;
IPAddress controllerIp;

uint32_t bootId = 0;
uint32_t lastControlUs = 0;
uint32_t lastTelemetryMs = 0;
uint32_t lastWifiRetryMs = 0;
uint32_t lastConsumedRevision = 0;
uint32_t controlOverruns = 0;
uint32_t udpReceived = 0;
uint32_t udpAccepted = 0;
uint32_t udpRejected = 0;
uint32_t manualSequence = 0;

bool udpStarted = false;
bool controllerKnown = false;
bool wifiWasConnected = false;
bool manualPwmActive = false;

char udpPacket[UDP_PACKET_MAX_BYTES + 1];
char serialLine[128];
size_t serialLength = 0;

constexpr uint32_t MANUAL_SESSION_ID = 0x4D414E55U;

bool isMoving() {
    return motionState.state == MotionState::VELOCITY ||
           motionState.state == MotionState::DISTANCE_ACTIVE ||
           motionState.state == MotionState::DISTANCE_BRAKING ||
           manualPwmActive;
}

void printHelp() {
    Serial.println("HELP STATUS PIN ENC ZERO AUTO MANUAL STOP");
    Serial.println("M1/M2/M3 <pwm> | ALL <p1> <p2> <p3>");
    Serial.println("VEL <vx_cm_s> <vy_cm_s> <w_rad_s>");
    Serial.println("MOVE <motion_id> <vx_cm_s> <vy_cm_s> <w_rad_s> <distance_cm>");
}

void printStatus() {
    Serial.println("===== STATUS =====");
    Serial.printf("MODE = %s\n", controllerModeName(safetyState.mode));
    Serial.printf("WiFi = %s", WiFi.status() == WL_CONNECTED ? "CONNECTED" : "DISCONNECTED");
    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("  IP=%s", WiFi.localIP().toString().c_str());
    }
    Serial.println();
    Serial.printf("UDP = %s  rx/accepted/rejected=%lu/%lu/%lu\n",
                  udpStarted ? "ON" : "OFF",
                  static_cast<unsigned long>(udpReceived),
                  static_cast<unsigned long>(udpAccepted),
                  static_cast<unsigned long>(udpRejected));
    Serial.printf("state=%u fault=%s session=%lu seq=%lu command_age_ms=",
                  static_cast<unsigned>(motionState.state),
                  faultCodeName(safetyState.fault),
                  static_cast<unsigned long>(commandAcceptance.sessionId),
                  static_cast<unsigned long>(commandAcceptance.lastSequence));
    if (commandAcceptance.hasSession) {
        Serial.println(static_cast<unsigned long>(millis() - commandAcceptance.lastAcceptedAtMs));
    } else {
        Serial.println("NONE");
    }
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        Serial.printf("M%u target=%.2f measured=%.2f pwm=%d count=%ld\n",
                      i + 1, targetWheelSpeed[i], measuredWheelSpeed[i],
                      motorEncoderState.currentPwm[i],
                      static_cast<long>(motorEncoderState.count[i]));
    }
    Serial.println("==================");
}

void printPins() {
    Serial.printf("STBY=%u\n", MOTOR_STBY_PIN);
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        Serial.printf("M%u IN1=%u IN2=%u PWM=%u ENC_A=%u ENC_B=%u\n",
                      i + 1, MOTOR_IN1_PINS[i], MOTOR_IN2_PINS[i],
                      MOTOR_PWM_PINS[i], ENCODER_A_PINS[i], ENCODER_B_PINS[i]);
    }
}

void printEncoders() {
    int32_t counts[MOTOR_COUNT];
    snapshotEncoderCounts(counts);
    Serial.printf("ENC M1=%ld M2=%ld M3=%ld\n", static_cast<long>(counts[0]),
                  static_cast<long>(counts[1]), static_cast<long>(counts[2]));
}

void beginUdpIfConnected() {
    if (!udpStarted && WiFi.status() == WL_CONNECTED) {
        udpStarted = commandUdp.begin(UDP_COMMAND_PORT) == 1;
        Serial.printf("[UDP] begin port=%u result=%s\n", UDP_COMMAND_PORT,
                      udpStarted ? "OK" : "FAIL");
    }
}

void beginWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.setAutoReconnect(true);
    if (strcmp(WIFI_SSID, "YOUR_WIFI_SSID") == 0 || WIFI_SSID[0] == '\0') {
        Serial.println("[WiFi] secrets.h is not configured");
        return;
    }
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    lastWifiRetryMs = millis();
    Serial.println("[WiFi] connecting with DHCP");
}

void serviceWiFi() {
    const bool connected = WiFi.status() == WL_CONNECTED;
    if (connected && !wifiWasConnected) {
        Serial.println("[WiFi] connected");
        Serial.printf("[WiFi] IP=%s\n", WiFi.localIP().toString().c_str());
        Serial.printf("[WiFi] gateway=%s\n", WiFi.gatewayIP().toString().c_str());
        Serial.printf("[WiFi] subnet=%s\n", WiFi.subnetMask().toString().c_str());
        beginUdpIfConnected();
    } else if (!connected && wifiWasConnected) {
        commandUdp.stop();
        udpStarted = false;
        Serial.println("[WiFi] disconnected");
    }
    wifiWasConnected = connected;

    const uint32_t nowMs = millis();
    if (!connected && WIFI_SSID[0] != '\0' &&
        strcmp(WIFI_SSID, "YOUR_WIFI_SSID") != 0 &&
        uint32_t(nowMs - lastWifiRetryMs) >= WIFI_RETRY_MS) {
        lastWifiRetryMs = nowMs;
        WiFi.reconnect();
    }
}

void rememberController(const IPAddress &remoteIp) {
    controllerIp = remoteIp;
    controllerKnown = true;
}

void handleAcceptedCommand(const NormalizedCommand &command,
                           bool fromNetwork, const IPAddress &remoteIp) {
    const AcceptanceResult result =
        acceptCommand(command, commandAcceptance, commandMailbox);
    if (result != AcceptanceResult::IGNORE_DUPLICATE_OR_STALE &&
        result != AcceptanceResult::FAULT_INVALID_COMMAND && fromNetwork) {
        rememberController(remoteIp);
    }

    switch (result) {
        case AcceptanceResult::ACCEPTED_MOTION:
            if (safetyState.fault != FaultCode::NONE) {
                clearMotionMailbox(commandMailbox);
            } else {
                ++udpAccepted;
            }
            break;
        case AcceptanceResult::ACCEPTED_LIVENESS:
            ++udpAccepted;
            tryResetFault(true, false, commandAcceptance.handshakeComplete);
            break;
        case AcceptanceResult::ACCEPTED_RESET:
            ++udpAccepted;
            if (safetyState.faultLatched &&
                safetyState.fault != FaultCode::NONE) {
                commandAcceptance.handshakeComplete = false;
            }
            tryResetFault(true, true, commandAcceptance.handshakeComplete);
            break;
        case AcceptanceResult::IMMEDIATE_STOP:
            ++udpAccepted;
            manualPwmActive = false;
            immediateStopNow(FaultCode::NONE, command.receivedAtMs,
                             commandMailbox, commandAcceptance);
            if (command.type == CommandType::STOP) {
                tryResetFault(true, false, commandAcceptance.handshakeComplete);
            }
            break;
        case AcceptanceResult::FAULT_SPEED_LIMIT:
            ++udpRejected;
            raiseFault(FaultCode::SPEED_LIMIT, command.receivedAtMs,
                       commandMailbox, commandAcceptance);
            break;
        case AcceptanceResult::FAULT_INVALID_COMMAND:
            ++udpRejected;
            raiseFault(FaultCode::BAD_PACKET, command.receivedAtMs,
                       commandMailbox, commandAcceptance);
            break;
        case AcceptanceResult::IGNORE_DUPLICATE_OR_STALE:
        case AcceptanceResult::NEEDS_HANDSHAKE:
            break;
    }
}

NormalizedCommand manualCommand(CommandType type) {
    NormalizedCommand command;
    command.type = type;
    command.sessionId = MANUAL_SESSION_ID;
    command.seq = ++manualSequence;
    command.status = type == CommandType::STOP ? CommandStatus::STOP
                                               : CommandStatus::RUN;
    command.receivedAtMs = millis();
    return command;
}

void enterManualMode() {
    immediateStopNow(FaultCode::NONE, millis(), commandMailbox,
                     commandAcceptance);
    commandAcceptance = CommandAcceptanceState{};
    safetyState.mode = ControllerMode::MANUAL_TEST;
    manualPwmActive = false;
    noteManualCommand(millis());
}

void enterNetworkMode() {
    immediateStopNow(FaultCode::NONE, millis(), commandMailbox,
                     commandAcceptance);
    commandAcceptance = CommandAcceptanceState{};
    safetyState.mode = ControllerMode::NETWORK;
    manualPwmActive = false;
}

void processSerialLine(char *line) {
    char commandName[12] = {};
    if (sscanf(line, "%11s", commandName) != 1) {
        return;
    }
    for (char *cursor = commandName; *cursor; ++cursor) {
        *cursor = static_cast<char>(toupper(*cursor));
    }

    if (strcmp(commandName, "HELP") == 0) {
        printHelp();
    } else if (strcmp(commandName, "STATUS") == 0) {
        printStatus();
    } else if (strcmp(commandName, "PIN") == 0) {
        printPins();
    } else if (strcmp(commandName, "ENC") == 0) {
        printEncoders();
    } else if (strcmp(commandName, "ZERO") == 0) {
        zeroEncoderReference();
        Serial.println("OK ZERO");
    } else if (strcmp(commandName, "AUTO") == 0) {
        enterNetworkMode();
        Serial.println("OK AUTO");
    } else if (strcmp(commandName, "MANUAL") == 0) {
        enterManualMode();
        Serial.println("OK MANUAL");
    } else if (strcmp(commandName, "STOP") == 0) {
        manualPwmActive = false;
        NormalizedCommand stop = manualCommand(CommandType::STOP);
        handleAcceptedCommand(stop, false, IPAddress());
        noteManualCommand(millis());
        Serial.println("OK STOP");
    } else if (strcmp(commandName, "M1") == 0 ||
               strcmp(commandName, "M2") == 0 ||
               strcmp(commandName, "M3") == 0) {
        int pwm = 0;
        if (sscanf(line, "%*s %d", &pwm) == 1) {
            enterManualMode();
            manualPwmActive = true;
            writeMotorPwm(static_cast<uint8_t>(commandName[1] - '1'), pwm);
            noteManualCommand(millis());
        }
    } else if (strcmp(commandName, "ALL") == 0) {
        int pwm[MOTOR_COUNT] = {};
        if (sscanf(line, "%*s %d %d %d", &pwm[0], &pwm[1], &pwm[2]) == 3) {
            enterManualMode();
            manualPwmActive = true;
            for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
                writeMotorPwm(i, pwm[i]);
            }
            noteManualCommand(millis());
        }
    } else if (strcmp(commandName, "VEL") == 0) {
        float vx = 0.0f, vy = 0.0f, w = 0.0f;
        if (sscanf(line, "%*s %f %f %f", &vx, &vy, &w) == 3) {
            enterManualMode();
            NormalizedCommand velocity = manualCommand(CommandType::VELOCITY);
            velocity.vxCmS = vx;
            velocity.vyCmS = vy;
            velocity.wRadS = w;
            handleAcceptedCommand(velocity, false, IPAddress());
            noteManualCommand(millis());
        }
    } else if (strcmp(commandName, "MOVE") == 0) {
        int motionId = -1;
        float vx = 0.0f, vy = 0.0f, w = 0.0f, distance = 0.0f;
        if (sscanf(line, "%*s %d %f %f %f %f", &motionId, &vx, &vy, &w,
                   &distance) == 5) {
            enterManualMode();
            NormalizedCommand move = manualCommand(CommandType::MOVE);
            move.motionId = motionId;
            move.vxCmS = vx;
            move.vyCmS = vy;
            move.wRadS = w;
            move.targetDistanceCm = distance;
            handleAcceptedCommand(move, false, IPAddress());
            noteManualCommand(millis());
        }
    } else {
        Serial.println("ERR unknown command; use HELP");
    }
}

void serviceSerial() {
    uint8_t charactersHandled = 0;
    while (Serial.available() > 0 && charactersHandled < 64) {
        ++charactersHandled;
        const char value = static_cast<char>(Serial.read());
        if (value == '\r') {
            continue;
        }
        if (value == '\n') {
            serialLine[serialLength] = '\0';
            processSerialLine(serialLine);
            serialLength = 0;
        } else if (serialLength + 1 < sizeof(serialLine)) {
            serialLine[serialLength++] = value;
        } else {
            serialLength = 0;
        }
    }
}

void serviceUdpRxBudgeted() {
    if (!udpStarted) {
        return;
    }
    const uint32_t startedUs = micros();
    uint8_t packetsHandled = 0;
    while (packetsHandled < UDP_PACKET_BUDGET &&
           uint32_t(micros() - startedUs) < UDP_TIME_BUDGET_US) {
        const int packetSize = commandUdp.parsePacket();
        if (packetSize <= 0) {
            break;
        }
        ++packetsHandled;
        ++udpReceived;
        const IPAddress remoteIp = commandUdp.remoteIP();
        if (packetSize > UDP_PACKET_MAX_BYTES) {
            while (commandUdp.available() > 0) {
                commandUdp.read();
            }
            ++udpRejected;
            raiseFault(FaultCode::BAD_PACKET, millis(), commandMailbox,
                       commandAcceptance);
            continue;
        }
        const int received = commandUdp.read(
            reinterpret_cast<uint8_t *>(udpPacket), UDP_PACKET_MAX_BYTES);
        if (received <= 0) {
            ++udpRejected;
            raiseFault(FaultCode::BAD_PACKET, millis(), commandMailbox,
                       commandAcceptance);
            continue;
        }
        udpPacket[received] = '\0';
        const DecodeResult decoded =
            decodePacket(udpPacket, static_cast<size_t>(received), millis());
        if (!decoded.ok) {
            ++udpRejected;
            raiseFault(FaultCode::BAD_PACKET, millis(), commandMailbox,
                       commandAcceptance);
            continue;
        }
        handleAcceptedCommand(decoded.command, true, remoteIp);
    }
}

void runControlTickIfDue() {
    const uint32_t nowUs = micros();
    const uint32_t elapsedUs = uint32_t(nowUs - lastControlUs);
    if (lastControlUs != 0 && elapsedUs < CONTROL_PERIOD_US) {
        return;
    }
    lastControlUs = nowUs;
    const float dtSec = elapsedUs == 0 ? CONTROL_PERIOD_US / 1000000.0f
                                      : elapsedUs / 1000000.0f;
    const uint32_t nowMs = millis();
    if (elapsedUs > CONTROL_PERIOD_US * 2U) {
        ++controlOverruns;
    }
    if (isMoving() && elapsedUs > static_cast<uint32_t>(MAX_CONTROL_DT_SEC * 1000000.0f)) {
        raiseFault(FaultCode::CONTROL_TIMING, nowMs, commandMailbox,
                   commandAcceptance);
        manualPwmActive = false;
        return;
    }

    const bool encoderValid = updateMeasuredWheelSpeed(
        constrain(dtSec, MIN_CONTROL_DT_SEC, MAX_CONTROL_DT_SEC));
    if (!encoderValid) {
        raiseFault(FaultCode::ENCODER_JUMP, nowMs, commandMailbox,
                   commandAcceptance);
        manualPwmActive = false;
        return;
    }
    if (updateWatchdogs(nowMs, WiFi.status() == WL_CONNECTED, isMoving(),
                        commandMailbox, commandAcceptance) != FaultCode::NONE) {
        manualPwmActive = false;
        return;
    }
    if (manualPwmActive) {
        return;
    }

    NormalizedCommand pending;
    if (takeLatestMotion(commandMailbox, lastConsumedRevision, pending)) {
        const MotionApplyResult applied = applyMotionCommand(pending);
        if (applied == MotionApplyResult::MOTION_ID_CHANGED ||
            applied == MotionApplyResult::INVALID_COMMAND) {
            raiseFault(FaultCode::BAD_PACKET, nowMs, commandMailbox,
                       commandAcceptance);
            return;
        }
    }

    float wheelDeltaCm[MOTOR_COUNT];
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        wheelDeltaCm[i] = wheelDistanceCm(
            motorEncoderState.deltaCount[i], ENCODER_COUNTS_PER_REV,
            WHEEL_RADIUS_CM);
    }
    const MotionUpdateResult updated =
        updateMotionState(wheelDeltaCm, encoderValid, dtSec);
    if (updated == MotionUpdateResult::PATH_DEVIATION) {
        raiseFault(FaultCode::PATH_DEVIATION, nowMs, commandMailbox,
                   commandAcceptance);
        return;
    }
    if (updated == MotionUpdateResult::ENCODER_INVALID) {
        raiseFault(FaultCode::ENCODER_INVALID, nowMs, commandMailbox,
                   commandAcceptance);
        return;
    }
    if (updated == MotionUpdateResult::ODOMETRY_INVALID) {
        raiseFault(FaultCode::ODOMETRY_INVALID, nowMs, commandMailbox,
                   commandAcceptance);
        return;
    }

    computeLimitedWheelTargets(
        constrain(dtSec, MIN_CONTROL_DT_SEC, MAX_CONTROL_DT_SEC));
    runWheelPid(constrain(dtSec, MIN_CONTROL_DT_SEC, MAX_CONTROL_DT_SEC));
    commandAcceptance.moving = isMoving();
}

void runTelemetryIfDue() {
    const uint32_t nowMs = millis();
    if (!udpStarted || !controllerKnown ||
        uint32_t(nowMs - lastTelemetryMs) < TELEMETRY_PERIOD_MS) {
        return;
    }
    lastTelemetryMs = nowMs;
    JsonDocument document;
    buildTelemetry(document, nowMs, bootId, commandAcceptance,
                   WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0,
                   controlOverruns);
    sendTelemetry(commandUdp, controllerIp, UDP_TELEMETRY_PORT, document);
}

}  // namespace

void setup() {
    Serial.begin(SERIAL_BAUD);
    initializeMotorOutputsSafe();
    attachEncoderInterrupts();
    resetMotionAfterFault();
    zeroEncoderReference();
    bootId = esp_random();
    if (bootId == 0) {
        bootId = 1;
    }
    beginWiFi();
    beginUdpIfConnected();
    enableMotorDriver();
    Serial.println("[BOOT] GRISE modular motion controller ready; motors stopped");
    printHelp();
}

void loop() {
    runControlTickIfDue();
    serviceSerial();
    serviceWiFi();
    serviceUdpRxBudgeted();
    runControlTickIfDue();
    runTelemetryIfDue();
    delay(0);
}
