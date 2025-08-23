#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

typedef uint8_t   UI_8;
typedef uint16_t  UI_16;
typedef int8_t    SI_8;
typedef int16_t   SI_16;

/* === Constants parameters for servo ===  */
#define SERVO_PWM_MIN           360
#define SERVO_PWM_MAX           2550
#define SERVO_FREQUENCY         50
#define MOTION_DURATION         150

/* === Constant parameters for queue === */
#define TRAJECTORY_QUEUE_SIZE   16
#define QUEUE_EMPTY             -1
#define INCREASEMENT            1

/* === Row and column of 2D matrix store initial value of servo === */
#define MAX_ANGLE       2
#define MAX_JOINT       6

/* === EEPROM configuration === */
#define EEPROM_SIZE     64
#define MAGIC_ADDR      0
#define MAGIC_VAL       0x27


/* === All joint configuration === */
#define BASE_JOINT      12
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

