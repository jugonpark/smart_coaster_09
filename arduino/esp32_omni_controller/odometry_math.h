#pragma once
#include <math.h>
#include <stdint.h>

// Inverse of wheel_i = -sin(angle_i)*vx + cos(angle_i)*vy + radius*w
// for wheel angles 0, 120, 240 degrees. Units: cm, radians.
struct BodyDelta {
    float dx;
    float dy;
    float dtheta;
};

constexpr float wheelDistanceCm(int64_t counts, float countsPerRev, float wheelRadiusCm) {
    return (float)counts * (2.0f * 3.14159265358979323846f * wheelRadiusCm) / countsPerRev;
}

constexpr BodyDelta forwardKinematics(float d1, float d2, float d3, float robotRadiusCm) {
    return {(d3 - d2) / 1.7320508075688772f,
            (2.0f * d1 - d2 - d3) / 3.0f,
            (d1 + d2 + d3) / (3.0f * robotRadiusCm)};
}

constexpr float goalProgressCm(const BodyDelta &delta, float ux, float uy) {
    return delta.dx * ux + delta.dy * uy;
}

constexpr float lateralErrorCm(const BodyDelta &delta, float ux, float uy) {
    return -delta.dx * uy + delta.dy * ux;
}

constexpr bool canStartDistanceGoal(bool armed, bool networkMode,
                                    int32_t incomingId, int32_t previousId) {
    return armed && networkMode && incomingId >= 0 && incomingId != previousId;
}

constexpr bool distanceGoalReached(float progress, float target, float tolerance) {
    return progress >= target - tolerance;
}

static_assert(forwardKinematics(0, -1.7320508f, 1.7320508f, 9).dx > 1.99f,
              "front motion must be positive X");
static_assert(forwardKinematics(1, -0.5f, -0.5f, 9).dy > 0.99f,
              "left motion must be positive Y");
static_assert(forwardKinematics(9, 9, 9, 9).dtheta == 1.0f,
              "positive wheel rotation must be positive heading");
static_assert(goalProgressCm(forwardKinematics(1, -0.5f, -0.5f, 9), 1, 0) == 0,
              "lateral drift must not count as goal progress");
static_assert(forwardKinematics(0, 1.7320508f, -1.7320508f, 9).dx < -1.99f,
              "back motion must be negative X");
static_assert(forwardKinematics(-1, 0.5f, 0.5f, 9).dy < -0.99f,
              "right motion must be negative Y");
static_assert(forwardKinematics(-0.7071068f, 0.9659258f, -0.258819f, 9).dx < -0.7f,
              "diagonal motion must preserve X");
static_assert(wheelDistanceCm(82313, 823.13f, 2.9f) > 1800.0f,
              "wheel count conversion must include circumference");
constexpr BodyDelta back29 = forwardKinematics(0, 25.1147367f, -25.1147367f, 9);
constexpr BodyDelta back30 = forwardKinematics(0, 25.9807621f, -25.9807621f, 9);
static_assert(goalProgressCm(back29, -1, 0) < 29.5f &&
              goalProgressCm(back30, -1, 0) >= 29.5f,
              "30 cm back goal changes state at the threshold");
static_assert(!distanceGoalReached(goalProgressCm(back29, -1, 0), 30, 0.5f) &&
              distanceGoalReached(goalProgressCm(back30, -1, 0), 30, 0.5f) &&
              distanceGoalReached(14.5f, 15, 0.5f),
              "15 and 30 cm stop thresholds");
static_assert(!canStartDistanceGoal(false, true, 12, 11) &&
              !canStartDistanceGoal(true, true, 11, 11) &&
              !canStartDistanceGoal(true, false, 12, 11) &&
              canStartDistanceGoal(true, true, 12, 11),
              "completed or canceled ID cannot restart; STOP must arm a new ID");
static_assert(goalProgressCm(forwardKinematics(1, -0.5f, -0.5f, 9), 1, 0) == 0 &&
              lateralErrorCm(forwardKinematics(1, -0.5f, -0.5f, 9), 1, 0) == 1,
              "lateral motion must stay separate from progress");
