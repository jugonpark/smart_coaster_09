#pragma once

#include "odometry_math.h"
#include "robot_config.h"

constexpr bool approximately(float left, float right, float tolerance = 0.001f) {
    return absFloat(left - right) <= tolerance;
}

constexpr WheelSpeeds SELF_X = inverseKinematicsBody(10.0f, 0.0f, 0.0f, 9.0f);
constexpr WheelSpeeds SELF_Y = inverseKinematicsBody(0.0f, 10.0f, 0.0f, 9.0f);
constexpr WheelSpeeds SELF_W = inverseKinematicsBody(0.0f, 0.0f, 1.0f, 9.0f);
constexpr BodyDelta SELF_X_BACK = forwardKinematics(SELF_X.value[0], SELF_X.value[1], SELF_X.value[2], 9.0f);
constexpr BodyDelta SELF_Y_BACK = forwardKinematics(SELF_Y.value[0], SELF_Y.value[1], SELF_Y.value[2], 9.0f);
constexpr BodyDelta SELF_W_BACK = forwardKinematics(SELF_W.value[0], SELF_W.value[1], SELF_W.value[2], 9.0f);

static_assert(approximately(inverseKinematicsBody(0, 0, 0, 9).value[0], 0), "zero body command must produce zero wheels");
static_assert(approximately(SELF_X.value[0], 0.0f) && approximately(SELF_X.value[1], -8.660254f, 0.001f) && approximately(SELF_X.value[2], 8.660254f, 0.001f), "positive X wheel targets must match 0/120/240 geometry");
static_assert(approximately(SELF_X_BACK.dx, 10.0f) && approximately(SELF_X_BACK.dy, 0.0f), "X IK/FK round trip");
static_assert(approximately(SELF_Y_BACK.dx, 0.0f) && approximately(SELF_Y_BACK.dy, 10.0f), "Y IK/FK round trip");
static_assert(approximately(SELF_W_BACK.dtheta, 1.0f), "rotation IK/FK round trip");
static_assert(approximately(goalProgressCm({3, 4, 0}, 1, 0), 3), "progress is parallel projection");
static_assert(approximately(lateralErrorCm({3, 4, 0}, 1, 0), 4), "lateral error is perpendicular projection");
static_assert(encoderDeltaLimitCounts(20, 2.9f, 898, 0.01f, 3) >= 29 && encoderDeltaLimitCounts(20, 2.9f, 898, 0.01f, 3) <= 31, "encoder jump limit must derive from physical speed");
static_assert(approximately(brakingSpeedLimit(2, 9), 6, 0.001f) && brakingSpeedLimit(2, 0) == 0, "braking bound must be sqrt(2*a*d)");
static_assert(brakingSpeedLimit(2, 16) > brakingSpeedLimit(2, 4), "braking speed must increase with remaining distance");
static_assert(approximately(slewTowards(0, 10, 20, 0.1f), 2), "positive slew is rate limited");
static_assert(approximately(slewTowards(1, -10, 20, 0.1f), -1), "negative slew is rate limited through zero");

