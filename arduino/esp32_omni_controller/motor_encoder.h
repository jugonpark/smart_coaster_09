#pragma once

#include <Arduino.h>
#include <stdint.h>

#include "robot_config.h"

struct MotorEncoderState {
    int32_t count[robot_config::MOTOR_COUNT];
    int32_t previousCount[robot_config::MOTOR_COUNT];
    float measuredWheelSpeed[robot_config::MOTOR_COUNT];
    int currentPwm[robot_config::MOTOR_COUNT];
    bool encoderValid[robot_config::MOTOR_COUNT];
    bool driverEnabled;
};

extern MotorEncoderState motorEncoderState;

void initializeMotorOutputsSafe();
void enableMotorDriver();
void disableMotorDriver();
void writeMotorPwm(uint8_t motorIndex, int signedPwm);
void stopAllMotorsImmediate();
void attachEncoderInterrupts();
void snapshotEncoderCounts(int32_t snapshot[robot_config::MOTOR_COUNT]);
bool updateMeasuredWheelSpeed(float dtSec);
void zeroEncoderReference();

