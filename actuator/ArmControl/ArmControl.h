#ifndef ARM_CONTROL_H
#define ARM_CONTROL_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

/* === Type Definitions === */
typedef uint8_t   UI_8;
typedef uint16_t  UI_16;
typedef int8_t    SI_8;
typedef int16_t   SI_16;
typedef int32_t   SI_32;

/* === Common Defines === */
#define D_FALSE     0
#define D_TRUE      1
#define OCLFRQ      27000000
#define SERCHNL     115200
#define WAITINIT    10
#define FX_SCALE    1000      /* scale factor for fixed-point */

/* === Servo Parameters === */
#define SERVO_PWM_MIN           360
#define SERVO_PWM_MAX           2550
#define SERVO_FREQUENCY         50
#define MOTION_DURATION         150
#define MED_DEVIDE              2
#define DELAY_TIME              500
#define MARK_UP_TIME            2000

/* === Queue Parameters === */
#define TRAJECTORY_QUEUE_SIZE   16
#define QUEUE_ERROR             -1 
#define INCREASE                1 
#define QUEUE_EMPTY             0

/* === Matrix/Array Dimensions === */
#define MAX_ANGLE       2
#define MAX_JOINT       6

/* === EEPROM Configuration === */
#define EEPROM_SIZE       64
#define MAGIC_ADDR        0
#define MAGIC_INIT_VAL    0x27
#define MAGIC_MODF_VAL    0x12
#define START_JOINT_ADDR  1
#define JOINT_BYTE_SIZE   2
#define MSB_OFFSET        0
#define LSB_OFFSET        1
#define SHIFT_BYTE        8
#define BYTE_MASK         0xFF

/* === System Input === */

#define D_OFF                       0
#define D_ON                        1

#define BASE_MAX_ANGLE_PIN          5
#define BASE_MIN_ANGLE_PIN          12

#define RIGHT_MAX_ANGLE_PIN         13
#define RIGHT_MIN_ANGLE_PIN         14

#define LEFT_MAX_ANGLE_PIN          18
#define LEFT_MIN_ANGLE_PIN          19 

#define TWIST_MAX_ANGLE_PIN         25
#define TWIST_MIN_ANGLE_PIN         26 

#define WRIST_MAX_ANGLE_PIN         27
#define WRIST_MIN_ANGLE_PIN         32 

#define FINGER_MAX_ANGLE_PIN        33
#define FINGER_MIN_ANGLE_PIN        34 

/* === Joint Channel Configuration === */
#define BASE_JOINT      0
#define RIGHT_JOINT     1
#define LEFT_JOINT      2
#define TWIST_JOINT     3
#define WRIST_JOINT     4
#define FINGER_JOINT    5

/* === Joint Factory Value === */

#define BASE_MAX_ANGLE          109
#define BASE_MIN_ANGLE          -109
#define BASE_ANGLE_VAL          0

#define RIGHT_MAX_ANGLE         218
#define RIGHT_MIN_ANGLE         0
#define RIGHT_ANGLE_VAL         37

#define LEFT_MAX_ANGLE          218
#define LEFT_MIN_ANGLE          0
#define LEFT_ANGLE_VAL          127

#define TWIST_MAX_ANGLE         218
#define TWIST_MIN_ANGLE         0
#define TWIST_ANGLE_VAL         109

#define WRIST_MAX_ANGLE         198
#define WRIST_MIN_ANGLE         0
#define WRIST_ANGLE_VAL         11

#define FINGER_MAX_ANGLE        206
#define FINGER_MIN_ANGLE        0
#define FINGER_ANGLE_VAL        108

/* === Arm Control API === */
namespace ArmControl {
  void initialize();
  void handleSerialCommands();
  void updateMotion();
}

#endif

