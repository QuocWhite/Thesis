#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// === Constants ===
#define SERVO_PWM_MIN      360
#define SERVO_PWM_MAX     2550
#define SERVO_FREQUENCY     50
#define TRAJECTORY_QUEUE_SIZE  16

#define BASE_JOINT     12
#define RIGHT_JOINT     0
#define LEFT_JOINT      1
#define TWIST_JOINT     8
#define WRIST_JOINT     7
#define FINGER_JOINT    5

namespace ArmControl {
  void initialize();
  void handleSerialCommands();
  void updateMotion();
  void reportCurrentJointPositions();
}

#endif

