#pragma once

#include <float.h>
#include <stdint.h>

struct BodyDelta {
    float dx;
    float dy;
    float dtheta;
};

struct WheelSpeeds {
    float value[3];
};

constexpr float absFloat(float value) {
    return value < 0.0f ? -value : value;
}

constexpr bool finiteFloat(float value) {
    return value == value && value <= FLT_MAX && value >= -FLT_MAX;
}

constexpr float sqrtPositive(float value) {
    if (!(value > 0.0f) || !finiteFloat(value)) {
        return 0.0f;
    }
    float estimate = value > 1.0f ? value : 1.0f;
    for (int i = 0; i < 16; ++i) {
        estimate = 0.5f * (estimate + value / estimate);
    }
    return estimate;
}

constexpr WheelSpeeds inverseKinematicsBody(float vxCmS, float vyCmS,
                                             float wRadS, float robotRadiusCm) {
    constexpr float HALF = 0.5f;
    constexpr float SQRT3_OVER_2 = 0.8660254037844386f;
    return {{{vyCmS + robotRadiusCm * wRadS,
              -SQRT3_OVER_2 * vxCmS - HALF * vyCmS + robotRadiusCm * wRadS,
              SQRT3_OVER_2 * vxCmS - HALF * vyCmS + robotRadiusCm * wRadS}}};
}

constexpr BodyDelta forwardKinematics(float d1, float d2, float d3,
                                      float robotRadiusCm) {
    return {(d3 - d2) / 1.7320508075688772f,
            (2.0f * d1 - d2 - d3) / 3.0f,
            (d1 + d2 + d3) / (3.0f * robotRadiusCm)};
}

constexpr float wheelDistanceCm(int64_t counts, float countsPerRev,
                                float wheelRadiusCm) {
    return countsPerRev > 0.0f
               ? static_cast<float>(counts) *
                     (2.0f * 3.14159265358979323846f * wheelRadiusCm) /
                     countsPerRev
               : 0.0f;
}

constexpr float goalProgressCm(const BodyDelta &delta, float ux, float uy) {
    return delta.dx * ux + delta.dy * uy;
}

constexpr float lateralErrorCm(const BodyDelta &delta, float ux, float uy) {
    return -delta.dx * uy + delta.dy * ux;
}

constexpr int64_t encoderDeltaLimitCounts(float wheelLimitCmS,
                                          float wheelRadiusCm,
                                          float countsPerRev,
                                          float dtSec,
                                          float safetyMargin) {
    if (!(wheelLimitCmS > 0.0f) || !(wheelRadiusCm > 0.0f) ||
        !(countsPerRev > 0.0f) || !(dtSec > 0.0f) ||
        !(safetyMargin >= 1.0f)) {
        return 0;
    }
    const float circumference = 2.0f * 3.14159265358979323846f * wheelRadiusCm;
    const float expected = wheelLimitCmS * dtSec * countsPerRev / circumference;
    return static_cast<int64_t>(expected * safetyMargin) + 1;
}

constexpr float brakingSpeedLimit(float decelerationCmS2, float remainingCm) {
    if (!finiteFloat(decelerationCmS2) || !finiteFloat(remainingCm) ||
        decelerationCmS2 <= 0.0f || remainingCm <= 0.0f) {
        return 0.0f;
    }
    return sqrtPositive(2.0f * decelerationCmS2 * remainingCm);
}

constexpr float slewTowards(float current, float target, float ratePerSec,
                            float dtSec) {
    if (!finiteFloat(current) || !finiteFloat(target) ||
        !(ratePerSec > 0.0f) || !(dtSec > 0.0f)) {
        return current;
    }
    const float maxStep = ratePerSec * dtSec;
    const float error = target - current;
    if (error > maxStep) {
        return current + maxStep;
    }
    if (error < -maxStep) {
        return current - maxStep;
    }
    return target;
}
