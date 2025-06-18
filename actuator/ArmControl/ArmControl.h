#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

namespace ArmControl {
  void initialize();
  void handleSerialCommands();
  void updateMotion();
  void reportCurrentJointPositions();
}

#endif

