#include "kinematics_motion.h"

#include <Arduino.h>
#include <math.h>

#include "motor_encoder.h"
#include "odometry_math.h"

using namespace robot_config;

namespace motion_control {

MotionControllerState motionState{};
float pidIntegral[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
float pidPreviousError[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
float measuredWheelSpeed[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
float targetWheelSpeed[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};

namespace {

bool isDistanceState(MotionState state) {
    return state == MotionState::DISTANCE_ACTIVE ||
           state == MotionState::DISTANCE_BRAKING ||
           state == MotionState::GOAL_REACHED;
}

float feedForwardPwm(uint8_t motorIndex, float targetSpeed) {
    if (!ENABLE_FEED_FORWARD || motorIndex >= MOTOR_COUNT || fabsf(targetSpeed) < 0.3f) {
        return 0.0f;
    }
    const float speed = fabsf(targetSpeed);
    // The LUT is not extrapolated. PID alone controls targets beyond the
    // measured feed-forward calibration range.
    if (speed > FF_SPEED_CM_S[FF_POINT_COUNT - 1]) {
        return 0.0f;
    }
    const float *table = targetSpeed >= 0.0f ? FF_PWM_POS[motorIndex] : FF_PWM_NEG[motorIndex];
    float pwm = table[FF_POINT_COUNT - 1];
    if (speed < FF_SPEED_CM_S[FF_POINT_COUNT - 1]) {
        for (uint8_t point = 0; point + 1 < FF_POINT_COUNT; ++point) {
            if (speed >= FF_SPEED_CM_S[point] && speed <= FF_SPEED_CM_S[point + 1]) {
                const float ratio = (speed - FF_SPEED_CM_S[point]) /
                                    (FF_SPEED_CM_S[point + 1] - FF_SPEED_CM_S[point]);
                pwm = table[point] + ratio * (table[point + 1] - table[point]);
                break;
            }
        }
    }
    return targetSpeed < 0.0f ? -pwm : pwm;
}

float slewAxis(float current, float target, float acceleration,
               float deceleration, float dtSec) {
    const bool acceleratingSameDirection =
        current * target >= 0.0f && fabsf(target) > fabsf(current);
    return slewTowards(current, target,
                       acceleratingSameDirection ? acceleration : deceleration,
                       dtSec);
}

void capLinear(float &vx, float &vy, float limit) {
    const float magnitude = hypotf(vx, vy);
    if (magnitude > limit && magnitude > 0.0f) {
        const float scale = limit / magnitude;
        vx *= scale;
        vy *= scale;
    }
}

void resetPidState() {
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        pidIntegral[i] = 0.0f;
        pidPreviousError[i] = 0.0f;
    }
}

}  // namespace

MotionApplyResult applyMotionCommand(const NormalizedCommand &command) {
    if (command.type == CommandType::VELOCITY) {
        motionState.commandStatus = command.status;
        motionState.desiredVxCmS = command.vxCmS;
        motionState.desiredVyCmS = command.vyCmS;
        motionState.desiredWRadS = command.wRadS;
        motionState.state = MotionState::VELOCITY;
        motionState.goalContentValid = false;
        motionState.motionId = -1;
        return MotionApplyResult::APPLIED;
    }
    if (command.type != CommandType::MOVE) {
        return MotionApplyResult::INVALID_COMMAND;
    }

    const float directionMagnitude = hypotf(command.vxCmS, command.vyCmS);
    if (!isfinite(directionMagnitude) || directionMagnitude <= 0.0f ||
        command.motionId < 0 || !isfinite(command.targetDistanceCm) ||
        command.targetDistanceCm <= 0.0f) {
        return MotionApplyResult::INVALID_COMMAND;
    }
    const float ux = command.vxCmS / directionMagnitude;
    const float uy = command.vyCmS / directionMagnitude;

    if (motionState.goalContentValid && command.motionId == motionState.motionId) {
        if (ux != motionState.goalUx || uy != motionState.goalUy ||
            directionMagnitude != motionState.goalRequestedSpeedCmS ||
            command.wRadS != motionState.goalRequestedWRadS ||
            command.status != motionState.goalRequestedStatus ||
            command.targetDistanceCm != motionState.goalTargetCm) {
            return MotionApplyResult::MOTION_ID_CHANGED;
        }
        return motionState.state == MotionState::GOAL_REACHED
                   ? MotionApplyResult::COMPLETED_ID
                   : MotionApplyResult::REFRESHED;
    }

    motionState.commandStatus = command.status;
    motionState.motionId = command.motionId;
    motionState.goalUx = ux;
    motionState.goalUy = uy;
    motionState.goalCruiseCmS = fminf(directionMagnitude, BODY_LINEAR_LIMIT_CM_S);
    motionState.goalRequestedSpeedCmS = directionMagnitude;
    motionState.goalRequestedWRadS = command.wRadS;
    motionState.goalRequestedStatus = command.status;
    motionState.goalTargetCm = command.targetDistanceCm;
    motionState.goalProgressCm = 0.0f;
    motionState.lateralErrorCm = 0.0f;
    motionState.remainingCm = command.targetDistanceCm;
    motionState.overshootCm = 0.0f;
    motionState.odomDxCm = 0.0f;
    motionState.odomDyCm = 0.0f;
    motionState.odomDthetaRad = 0.0f;
    motionState.desiredWRadS = command.wRadS;
    motionState.goalContentValid = true;
    motionState.overshootStopped = false;
    motionState.state = MotionState::DISTANCE_ACTIVE;
    return MotionApplyResult::APPLIED;
}

MotionUpdateResult updateMotionState(const float wheelDeltaCm[MOTOR_COUNT],
                                    bool encoderValid, float dtSec) {
    if (!encoderValid) {
        return MotionUpdateResult::ENCODER_INVALID;
    }
    if (wheelDeltaCm == nullptr || !isfinite(dtSec) || dtSec <= 0.0f) {
        return MotionUpdateResult::ODOMETRY_INVALID;
    }
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        if (!isfinite(wheelDeltaCm[i])) {
            return MotionUpdateResult::ODOMETRY_INVALID;
        }
    }

    const BodyDelta body = forwardKinematics(
        wheelDeltaCm[0], wheelDeltaCm[1], wheelDeltaCm[2], ROBOT_RADIUS_CM);
    const float midpointHeading = motionState.odomDthetaRad + 0.5f * body.dtheta;
    const float worldDx = cosf(midpointHeading) * body.dx - sinf(midpointHeading) * body.dy;
    const float worldDy = sinf(midpointHeading) * body.dx + cosf(midpointHeading) * body.dy;
    motionState.odomDxCm += worldDx;
    motionState.odomDyCm += worldDy;
    motionState.odomDthetaRad += body.dtheta;

    if (!isDistanceState(motionState.state) || !motionState.goalContentValid) {
        return MotionUpdateResult::OK;
    }
    const BodyDelta local{motionState.odomDxCm, motionState.odomDyCm,
                          motionState.odomDthetaRad};
    motionState.goalProgressCm =
        ::goalProgressCm(local, motionState.goalUx, motionState.goalUy);
    motionState.lateralErrorCm =
        ::lateralErrorCm(local, motionState.goalUx, motionState.goalUy);
    motionState.remainingCm = motionState.goalTargetCm - motionState.goalProgressCm;

    if (fabsf(motionState.lateralErrorCm) > PATH_DEVIATION_LIMIT_CM) {
        return MotionUpdateResult::PATH_DEVIATION;
    }
    if (motionState.remainingCm < 0.0f) {
        motionState.overshootCm = -motionState.remainingCm;
        motionState.overshootStopped = true;
        cancelMotionImmediate(MotionState::STOPPED);
        motionState.goalContentValid = true;
        motionState.overshootStopped = true;
        return MotionUpdateResult::OUTPUT_INHIBITED;
    }

    float maximumMeasuredSpeed = 0.0f;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        maximumMeasuredSpeed = fmaxf(maximumMeasuredSpeed, fabsf(measuredWheelSpeed[i]));
    }
    const bool reached = fabsf(motionState.remainingCm) <= DISTANCE_TOLERANCE_CM &&
                         maximumMeasuredSpeed <= GOAL_STOP_SPEED_CM_S &&
                         fabsf(motionState.lateralErrorCm) <= LATERAL_TOLERANCE_CM;
    if (reached) {
        cancelMotionImmediate(MotionState::GOAL_REACHED);
        motionState.goalContentValid = true;
        return MotionUpdateResult::GOAL_COMPLETE;
    }
    return MotionUpdateResult::OK;
}

void computeLimitedWheelTargets(float dtSec) {
    float requestedVx = 0.0f;
    float requestedVy = 0.0f;
    float requestedW = 0.0f;

    if (motionState.state == MotionState::VELOCITY) {
        requestedVx = motionState.desiredVxCmS;
        requestedVy = motionState.desiredVyCmS;
        requestedW = motionState.desiredWRadS;
    } else if (motionState.state == MotionState::DISTANCE_ACTIVE ||
               motionState.state == MotionState::DISTANCE_BRAKING) {
        float allowedSpeed = fminf(
            motionState.goalCruiseCmS,
            brakingSpeedLimit(BODY_DECEL_CM_S2, motionState.remainingCm));
        if (motionState.remainingCm > DISTANCE_TOLERANCE_CM &&
            allowedSpeed < APPROACH_SPEED_CM_S) {
            allowedSpeed = APPROACH_SPEED_CM_S;
        }
        if (allowedSpeed < motionState.goalCruiseCmS) {
            motionState.state = MotionState::DISTANCE_BRAKING;
        }
        if (motionState.overshootStopped || motionState.remainingCm <= 0.0f) {
            allowedSpeed = 0.0f;
        }
        requestedVx = motionState.goalUx * allowedSpeed;
        requestedVy = motionState.goalUy * allowedSpeed;
        requestedW = motionState.desiredWRadS;
    }

    const float linearLimit = motionState.commandStatus == CommandStatus::SLOW
                                  ? SLOW_LINEAR_LIMIT_CM_S
                                  : BODY_LINEAR_LIMIT_CM_S;
    const float angularLimit = motionState.commandStatus == CommandStatus::SLOW
                                   ? SLOW_ANGULAR_LIMIT_RAD_S
                                   : ANGULAR_LIMIT_RAD_S;
    capLinear(requestedVx, requestedVy, linearLimit);
    requestedW = constrain(requestedW, -angularLimit, angularLimit);

    motionState.limitedVxCmS = slewAxis(
        motionState.limitedVxCmS, requestedVx, BODY_ACCEL_CM_S2,
        BODY_DECEL_CM_S2, dtSec);
    motionState.limitedVyCmS = slewAxis(
        motionState.limitedVyCmS, requestedVy, BODY_ACCEL_CM_S2,
        BODY_DECEL_CM_S2, dtSec);
    motionState.limitedWRadS = slewAxis(
        motionState.limitedWRadS, requestedW, ANGULAR_ACCEL_RAD_S2,
        ANGULAR_DECEL_RAD_S2, dtSec);
    capLinear(motionState.limitedVxCmS, motionState.limitedVyCmS, linearLimit);

    WheelSpeeds desired = inverseKinematicsBody(
        motionState.limitedVxCmS, motionState.limitedVyCmS,
        motionState.limitedWRadS, ROBOT_RADIUS_CM);
    float maximum = 0.0f;
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        maximum = fmaxf(maximum, fabsf(desired.value[i]));
    }
    if (maximum > WHEEL_SPEED_LIMIT_CM_S) {
        const float scale = WHEEL_SPEED_LIMIT_CM_S / maximum;
        for (float &value : desired.value) {
            value *= scale;
        }
    }
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        targetWheelSpeed[i] = slewAxis(
            targetWheelSpeed[i], desired.value[i], WHEEL_ACCEL_CM_S2,
            WHEEL_DECEL_CM_S2, dtSec);
    }
}

void runWheelPid(float dtSec) {
    dtSec = constrain(dtSec, MIN_CONTROL_DT_SEC, MAX_CONTROL_DT_SEC);
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        measuredWheelSpeed[i] = motorEncoderState.measuredWheelSpeed[i];
        const float target = targetWheelSpeed[i];
        if (fabsf(target) < 0.3f) {
            pidIntegral[i] = 0.0f;
            pidPreviousError[i] = 0.0f;
            writeMotorPwm(i, 0);
            continue;
        }

        float feedForward = feedForwardPwm(i, target);
        if (fabsf(motionState.limitedVxCmS) < 0.1f &&
            fabsf(motionState.limitedVyCmS) < 0.1f &&
            fabsf(motionState.limitedWRadS) > 0.01f) {
            feedForward *= 0.75f;
        }

        const float error = target - measuredWheelSpeed[i];
        const float p = PID_KP * error;
        const float candidateIntegral = pidIntegral[i] + error * dtSec;
        const float iTerm = PID_KI * candidateIntegral;
        const float d = PID_KD * (error - pidPreviousError[i]) / dtSec;
        pidPreviousError[i] = error;
        float output = feedForward + p + iTerm + d;
        if (output > -PWM_MAX && output < PWM_MAX) {
            pidIntegral[i] = candidateIntegral;
        } else {
            output = feedForward + p + PID_KI * pidIntegral[i] + d;
        }
        output = constrain(output, -(float)PWM_MAX, (float)PWM_MAX);

        int pwm = static_cast<int>(lroundf(output));
        if (pwm != 0 && pwm > 0 && pwm < PWM_DEADZONE_POS[i]) {
            pwm = PWM_DEADZONE_POS[i];
        } else if (pwm != 0 && pwm < 0 && -pwm < PWM_DEADZONE_NEG[i]) {
            pwm = -PWM_DEADZONE_NEG[i];
        }
        writeMotorPwm(i, pwm);
    }
}

void cancelMotionImmediate(MotionState finalState) {
    stopAllMotorsImmediate();
    for (uint8_t i = 0; i < MOTOR_COUNT; ++i) {
        targetWheelSpeed[i] = 0.0f;
    }
    resetPidState();
    motionState.desiredVxCmS = 0.0f;
    motionState.desiredVyCmS = 0.0f;
    motionState.desiredWRadS = 0.0f;
    motionState.limitedVxCmS = 0.0f;
    motionState.limitedVyCmS = 0.0f;
    motionState.limitedWRadS = 0.0f;
    motionState.commandStatus = CommandStatus::STOP;
    motionState.state = finalState;
}

void resetMotionAfterFault() {
    cancelMotionImmediate(MotionState::STOPPED);
    motionState = MotionControllerState{};
}

}  // namespace motion_control
