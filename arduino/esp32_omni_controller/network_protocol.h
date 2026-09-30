#pragma once

#include <Arduino.h>
#include <stddef.h>
#include <stdint.h>

enum class CommandType : uint8_t {
    VELOCITY,
    MOVE,
    STOP,
    HEARTBEAT,
    RESET_FAULT,
};

enum class CommandStatus : uint8_t {
    RUN,
    SLOW,
    STOP,
};

struct NormalizedCommand {
    CommandType type = CommandType::STOP;
    uint32_t sessionId = 0;
    uint32_t seq = 0;
    CommandStatus status = CommandStatus::STOP;
    float vxCmS = 0.0f;
    float vyCmS = 0.0f;
    float wRadS = 0.0f;
    int32_t motionId = -1;
    float targetDistanceCm = 0.0f;
    uint32_t receivedAtMs = 0;
    bool legacy = false;
};

enum class DecodeError : uint8_t {
    NONE,
    EMPTY_PACKET,
    PACKET_TOO_LARGE,
    MALFORMED_JSON,
    INVALID_TYPE,
    INVALID_SESSION,
    INVALID_SEQUENCE,
    INVALID_STATUS,
    INVALID_VALUE,
    PARTIAL_DISTANCE,
    LEGACY_DISABLED,
};

struct DecodeResult {
    bool ok = false;
    DecodeError error = DecodeError::NONE;
    NormalizedCommand command{};
};

enum class AcceptanceResult : uint8_t {
    ACCEPTED_MOTION,
    ACCEPTED_LIVENESS,
    ACCEPTED_RESET,
    IGNORE_DUPLICATE_OR_STALE,
    NEEDS_HANDSHAKE,
    IMMEDIATE_STOP,
    FAULT_SPEED_LIMIT,
    FAULT_INVALID_COMMAND,
};

struct CommandMailbox {
    NormalizedCommand command{};
    uint32_t localRevision = 0;
    bool occupied = false;
};

struct CommandAcceptanceState {
    uint32_t sessionId = 0;
    uint32_t lastSequence = 0;
    uint32_t lastAcceptedAtMs = 0;
    bool hasSession = false;
    bool hasSequence = false;
    bool handshakeComplete = false;
    bool moving = false;
};

DecodeResult decodePacket(const char *data, size_t length, uint32_t receivedAtMs);
AcceptanceResult acceptCommand(const NormalizedCommand &command,
                               CommandAcceptanceState &state,
                               CommandMailbox &mailbox);
bool takeLatestMotion(const CommandMailbox &mailbox, uint32_t &lastTakenRevision,
                      NormalizedCommand &commandOut);
void clearMotionMailbox(CommandMailbox &mailbox);

