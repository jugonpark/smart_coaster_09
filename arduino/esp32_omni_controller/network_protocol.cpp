#include "network_protocol.h"

#include <ArduinoJson.h>
#include <math.h>
#include <string.h>

#include "robot_config.h"

using namespace robot_config;

namespace {

DecodeResult decodeFailure(DecodeError error) {
    DecodeResult result;
    result.error = error;
    return result;
}

bool finiteNumber(JsonObjectConst object, const char *key, float &value,
                  bool required = true) {
    JsonVariantConst field = object[key];
    if (field.isNull()) {
        if (required) {
            return false;
        }
        value = 0.0f;
        return true;
    }
    if (!field.is<float>() || field.is<bool>()) {
        return false;
    }
    value = field.as<float>();
    return isfinite(value);
}

bool parseStatus(JsonObjectConst object, CommandStatus &status, bool required) {
    const char *text = object["status"] | static_cast<const char *>(nullptr);
    if (text == nullptr) {
        return !required;
    }
    if (strcmp(text, "RUN") == 0) {
        status = CommandStatus::RUN;
        return true;
    }
    if (strcmp(text, "SLOW") == 0) {
        status = CommandStatus::SLOW;
        return true;
    }
    if (strcmp(text, "STOP") == 0) {
        status = CommandStatus::STOP;
        return true;
    }
    return false;
}

bool parseMotion(JsonObjectConst object, NormalizedCommand &command) {
    return finiteNumber(object, "vx", command.vxCmS) &&
           finiteNumber(object, "vy", command.vyCmS) &&
           finiteNumber(object, "w", command.wRadS) &&
           parseStatus(object, command.status, true) &&
           command.status != CommandStatus::STOP;
}

DecodeResult decodeTyped(JsonObjectConst root, uint32_t receivedAtMs) {
    if (!root["session_id"].is<uint32_t>() || root["session_id"].as<uint32_t>() == 0) {
        return decodeFailure(DecodeError::INVALID_SESSION);
    }
    if (!root["seq"].is<uint32_t>()) {
        return decodeFailure(DecodeError::INVALID_SEQUENCE);
    }

    NormalizedCommand command;
    command.sessionId = root["session_id"].as<uint32_t>();
    command.seq = root["seq"].as<uint32_t>();
    command.receivedAtMs = receivedAtMs;
    const char *type = root["type"] | static_cast<const char *>(nullptr);
    if (type == nullptr) {
        return decodeFailure(DecodeError::INVALID_TYPE);
    }

    if (strcmp(type, "stop") == 0) {
        command.type = CommandType::STOP;
        command.status = CommandStatus::STOP;
    } else if (strcmp(type, "heartbeat") == 0) {
        command.type = CommandType::HEARTBEAT;
    } else if (strcmp(type, "reset_fault") == 0) {
        command.type = CommandType::RESET_FAULT;
    } else if (strcmp(type, "cmd_vel") == 0) {
        command.type = CommandType::VELOCITY;
        if (!parseMotion(root, command)) {
            return decodeFailure(DecodeError::INVALID_VALUE);
        }
    } else if (strcmp(type, "cmd_move") == 0) {
        command.type = CommandType::MOVE;
        if (!parseMotion(root, command) || !root["motion_id"].is<int32_t>() ||
            root["motion_id"].as<int32_t>() < 0 ||
            !finiteNumber(root, "target_distance_cm", command.targetDistanceCm) ||
            command.targetDistanceCm <= 0.0f) {
            return decodeFailure(DecodeError::INVALID_VALUE);
        }
        command.motionId = root["motion_id"].as<int32_t>();
    } else {
        return decodeFailure(DecodeError::INVALID_TYPE);
    }

    DecodeResult result;
    result.ok = true;
    result.command = command;
    return result;
}

DecodeResult decodeLegacy(JsonObjectConst root, uint32_t receivedAtMs) {
    if (!ENABLE_LEGACY_PROTOCOL) {
        return decodeFailure(DecodeError::LEGACY_DISABLED);
    }
    if (!root["seq"].is<uint32_t>()) {
        return decodeFailure(DecodeError::INVALID_SEQUENCE);
    }

    NormalizedCommand command;
    command.sessionId = LEGACY_SESSION_ID;
    command.seq = root["seq"].as<uint32_t>();
    command.receivedAtMs = receivedAtMs;
    command.legacy = true;
    if (!parseStatus(root, command.status, true)) {
        return decodeFailure(DecodeError::INVALID_STATUS);
    }
    if (command.status == CommandStatus::STOP) {
        command.type = CommandType::STOP;
    } else {
        if (!parseMotion(root, command)) {
            return decodeFailure(DecodeError::INVALID_VALUE);
        }
        const bool hasMotionId = !root["motion_id"].isNull();
        const bool hasDistance = !root["target_distance_cm"].isNull();
        if (hasMotionId != hasDistance) {
            return decodeFailure(DecodeError::PARTIAL_DISTANCE);
        }
        if (hasMotionId) {
            if (!root["motion_id"].is<int32_t>() || root["motion_id"].as<int32_t>() < 0 ||
                !finiteNumber(root, "target_distance_cm", command.targetDistanceCm) ||
                command.targetDistanceCm <= 0.0f) {
                return decodeFailure(DecodeError::INVALID_VALUE);
            }
            command.type = CommandType::MOVE;
            command.motionId = root["motion_id"].as<int32_t>();
        } else {
            command.type = CommandType::VELOCITY;
        }
    }

    DecodeResult result;
    result.ok = true;
    result.command = command;
    return result;
}

bool violatesAbsoluteLimits(const NormalizedCommand &command) {
    if (command.type != CommandType::VELOCITY && command.type != CommandType::MOVE) {
        return false;
    }
    const float linear = hypotf(command.vxCmS, command.vyCmS);
    return !isfinite(linear) || linear > HARD_LINEAR_LIMIT_CM_S ||
           fabsf(command.wRadS) > HARD_ANGULAR_LIMIT_RAD_S ||
           (command.type == CommandType::MOVE &&
            command.targetDistanceCm > MAX_GOAL_DISTANCE_CM);
}

bool validNormalizedCommand(const NormalizedCommand &command) {
    if (command.sessionId == 0) {
        return false;
    }
    if (command.type != CommandType::VELOCITY && command.type != CommandType::MOVE) {
        return true;
    }
    if (!isfinite(command.vxCmS) || !isfinite(command.vyCmS) ||
        !isfinite(command.wRadS) || command.status == CommandStatus::STOP) {
        return false;
    }
    if (command.type == CommandType::MOVE) {
        return command.motionId >= 0 && isfinite(command.targetDistanceCm) &&
               command.targetDistanceCm > 0.0f;
    }
    return true;
}

}  // namespace

DecodeResult decodePacket(const char *data, size_t length, uint32_t receivedAtMs) {
    if (data == nullptr || length == 0) {
        return decodeFailure(DecodeError::EMPTY_PACKET);
    }
    if (length > UDP_PACKET_MAX_BYTES) {
        return decodeFailure(DecodeError::PACKET_TOO_LARGE);
    }

    JsonDocument document;
    const DeserializationError error = deserializeJson(document, data, length);
    if (error) {
        return decodeFailure(DecodeError::MALFORMED_JSON);
    }
    JsonObjectConst root = document.as<JsonObjectConst>();
    if (root.isNull()) {
        return decodeFailure(DecodeError::MALFORMED_JSON);
    }
    return root["type"].isNull() ? decodeLegacy(root, receivedAtMs)
                                 : decodeTyped(root, receivedAtMs);
}

AcceptanceResult acceptCommand(const NormalizedCommand &command,
                               CommandAcceptanceState &state,
                               CommandMailbox &mailbox) {
    if (!validNormalizedCommand(command)) {
        return AcceptanceResult::FAULT_INVALID_COMMAND;
    }

    if (!state.hasSession || state.sessionId != command.sessionId) {
        const bool wasMoving = state.moving;
        state.sessionId = command.sessionId;
        state.lastSequence = command.seq;
        state.lastAcceptedAtMs = command.receivedAtMs;
        state.hasSession = true;
        state.hasSequence = true;
        state.handshakeComplete = !wasMoving;
        if (wasMoving) {
            state.moving = false;
            clearMotionMailbox(mailbox);
            return AcceptanceResult::IMMEDIATE_STOP;
        }
    } else {
        // Duplicate/stale classification deliberately occurs only here,
        // after decode has produced a complete NormalizedCommand.
        if (state.hasSequence && command.seq <= state.lastSequence) {
            const bool legacyResyncStop =
                command.legacy && command.sessionId == LEGACY_SESSION_ID &&
                !state.moving && command.type == CommandType::STOP;
            if (!legacyResyncStop) {
                return AcceptanceResult::IGNORE_DUPLICATE_OR_STALE;
            }
        }
        state.lastSequence = command.seq;
        state.hasSequence = true;
        state.lastAcceptedAtMs = command.receivedAtMs;
    }

    if (violatesAbsoluteLimits(command)) {
        state.moving = false;
        clearMotionMailbox(mailbox);
        return AcceptanceResult::FAULT_SPEED_LIMIT;
    }

    switch (command.type) {
        case CommandType::STOP:
            state.handshakeComplete = true;
            state.moving = false;
            clearMotionMailbox(mailbox);
            return AcceptanceResult::IMMEDIATE_STOP;
        case CommandType::HEARTBEAT:
            state.handshakeComplete = true;
            return AcceptanceResult::ACCEPTED_LIVENESS;
        case CommandType::RESET_FAULT:
            return AcceptanceResult::ACCEPTED_RESET;
        case CommandType::VELOCITY:
        case CommandType::MOVE:
            if (!state.handshakeComplete) {
                return AcceptanceResult::NEEDS_HANDSHAKE;
            }
            mailbox.command = command;
            ++mailbox.localRevision;
            if (mailbox.localRevision == 0) {
                ++mailbox.localRevision;
            }
            mailbox.occupied = true;
            state.moving = true;
            return AcceptanceResult::ACCEPTED_MOTION;
    }
    return AcceptanceResult::FAULT_INVALID_COMMAND;
}

bool takeLatestMotion(const CommandMailbox &mailbox, uint32_t &lastTakenRevision,
                      NormalizedCommand &commandOut) {
    if (!mailbox.occupied || mailbox.localRevision == lastTakenRevision) {
        return false;
    }
    commandOut = mailbox.command;
    lastTakenRevision = mailbox.localRevision;
    return true;
}

void clearMotionMailbox(CommandMailbox &mailbox) {
    mailbox.occupied = false;
    mailbox.command = NormalizedCommand{};
}
