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

/* === Common Defines === */
#define D_FALSE     0
#define D_TRUE      1
#define OCLFRQ      27000000
#define SERCHNL     115200
#define WAITINIT    10

/* === Servo Parameters === */
#define SERVO_PWM_MIN           360
#define SERVO_PWM_MAX           2550
#define SERVO_FREQUENCY         50
#define MOTION_DURATION         150
#define MED_DEVIDE              2
#define DELAY_TIME              500

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

/* === Joint Channel Configuration === */
#define BASE_JOINT      12
#define RIGHT_JOINT     0
#define LEFT_JOINT      1
#define TWIST_JOINT     8
#define WRIST_JOINT     7
#define FINGER_JOINT    5

/* === Arm Control API === */
namespace ArmControl {
  void initialize();
  void handleSerialCommands();
  void updateMotion();
}

#endif

