#pragma once

#include <Arduino.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <stdint.h>

#include "network_protocol.h"

enum class FaultCode : uint8_t {
    NONE,
    WIFI_LOSS,
    CMD_TIMEOUT,
    BAD_PACKET,
    SPEED_LIMIT,
    ENCODER_JUMP,
    ENCODER_INVALID,
    ODOMETRY_INVALID,
    PATH_DEVIATION,
    CONTROL_TIMING,
};

enum class ControllerMode : uint8_t {
    NETWORK,
    MANUAL_TEST,
};

struct SafetyState {
    FaultCode fault = FaultCode::NONE;
    ControllerMode mode = ControllerMode::NETWORK;
    uint32_t faultAtMs = 0;
    uint32_t lastManualCommandMs = 0;
    bool faultLatched = false;
    bool resetAuthorized = false;
};

extern SafetyState safetyState;

void immediateStopNow(FaultCode cause, uint32_t nowMs,
                      CommandMailbox &mailbox,
                      CommandAcceptanceState &acceptance);
void raiseFault(FaultCode cause, uint32_t nowMs,
                CommandMailbox &mailbox,
                CommandAcceptanceState &acceptance);
bool tryResetFault(bool conditionClear, bool explicitReset,
                   bool handshakeComplete);
FaultCode updateWatchdogs(uint32_t nowMs, bool wifiConnected, bool moving,
                          CommandMailbox &mailbox,
                          CommandAcceptanceState &acceptance);
void noteManualCommand(uint32_t nowMs);
void buildTelemetry(JsonDocument &document, uint32_t nowMs, uint32_t bootId,
                    const CommandAcceptanceState &acceptance,
                    int32_t wifiRssi, uint32_t controlOverruns);
bool sendTelemetry(WiFiUDP &udp, const IPAddress &controllerIp,
                   uint16_t controllerPort, JsonDocument &document);
const char *faultCodeName(FaultCode fault);
const char *controllerModeName(ControllerMode mode);
