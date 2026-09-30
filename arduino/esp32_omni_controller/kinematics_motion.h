#pragma once

#include <stdint.h>

#include "network_protocol.h"
#include "robot_config.h"

namespace motion_control {

enum class MotionState : uint8_t {
    STOPPED,
    VELOCITY,
    DISTANCE_ACTIVE,
    DISTANCE_BRAKING,
    GOAL_REACHED,
    FAULT,
};

enum class MotionApplyResult : uint8_t {
    APPLIED,
    REFRESHED,
    COMPLETED_ID,
    MOTION_ID_CHANGED,
    INVALID_COMMAND,
};

enum class MotionUpdateResult : uint8_t {
    OK,
    GOAL_COMPLETE,
    PATH_DEVIATION,
    ENCODER_INVALID,
    ODOMETRY_INVALID,
};

struct MotionControllerState {
    MotionState state = MotionState::STOPPED;
    CommandStatus commandStatus = CommandStatus::STOP;
    float desiredVxCmS = 0.0f;
    float desiredVyCmS = 0.0f;
    float desiredWRadS = 0.0f;
    float limitedVxCmS = 0.0f;
    float limitedVyCmS = 0.0f;
    float limitedWRadS = 0.0f;
    int32_t motionId = -1;
    float goalUx = 0.0f;
    float goalUy = 0.0f;
    float goalCruiseCmS = 0.0f;
    float goalTargetCm = 0.0f;
    float goalProgressCm = 0.0f;
    float lateralErrorCm = 0.0f;
    float remainingCm = 0.0f;
    float overshootCm = 0.0f;
    float odomDxCm = 0.0f;
    float odomDyCm = 0.0f;
    float odomDthetaRad = 0.0f;
    bool goalContentValid = false;
    bool overshootStopped = false;
};

extern MotionControllerState motionState;
extern float pidIntegral[robot_config::MOTOR_COUNT];
extern float pidPreviousError[robot_config::MOTOR_COUNT];
extern float measuredWheelSpeed[robot_config::MOTOR_COUNT];
extern float targetWheelSpeed[robot_config::MOTOR_COUNT];

MotionApplyResult applyMotionCommand(const NormalizedCommand &command);
MotionUpdateResult updateMotionState(
    const float wheelDeltaCm[robot_config::MOTOR_COUNT], bool encoderValid,
    float dtSec);
void computeLimitedWheelTargets(float dtSec);
void runWheelPid(float dtSec);
void cancelMotionImmediate(MotionState finalState = MotionState::STOPPED);
void resetMotionAfterFault();

}  // namespace motion_control

