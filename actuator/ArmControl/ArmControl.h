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
typedef void (*CommandFunc)();

/* === Common Defines === */

#define D_FALSE     0
#define D_TRUE      1
#define OCLFRQ      27000000
#define SERCHNL     115200
#define WAITINIT    10
#define FX_SCALE    1000      /* scale factor for fixed-point */

/* === Servo Parameters === */

#define SERVO_MIN_TARGET        320
#define SERVO_MAX_TARGET        2570
#define SERVO_FREQUENCY         50
#define MOTION_DURATION         150
#define MED_DEVIDE              2
#define DELAY_TIME              500
#define MARK_UP_TIME            2000
#define SERVO_MIN_SOURCE        0
#define SERVO_MAX_SOURCE        180

/* === Queue Parameters === */

#define TRAJECTORY_QUEUE_SIZE   16
#define QUEUE_ERROR             -1
#define INCREASE                1
#define FIRST_INPUT             1
#define QUEUE_EMPTY             0
#define UNAVAIL_SER             0

/* === Matrix/Array Dimensions === */

#define MAX_ANGLE       2
#define FIRST_JOINT     0
#define FIRST_ANGLE     0

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

#define PIN_MIN                     0
#define PIN_MAX                     1
#define STEP_SIZE                   5
#define DELAY_SET                   10
#define D_OFF                       0
#define D_ON                        1
#define D_NG                        -1

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
#define FINGER_MIN_ANGLE_PIN        4 

/* === Joint Channel Configuration === */

#define BASE_JOINT      0
#define RIGHT_JOINT     1
#define LEFT_JOINT      2
#define TWIST_JOINT     3
#define WRIST_JOINT     4
#define FINGER_JOINT    5

/* === Joint Factory Value === */

#define COMMON_MIN_ANGLE             400          /* 0     */
#define COMMON_MAX_ANGLE             2450         /* 180   */

#define FACT_BASE_MIN_ANGLE          400          /* 0     */
#define FACT_BASE_MAX_ANGLE          2450         /* 180   */
#define FACT_BASE_ANGLE_VAL          1455         /* 1455  */

#define FACT_RIGHT_MIN_ANGLE         400          /* 0     */
#define FACT_RIGHT_MAX_ANGLE         2450         /* 180   */
#define FACT_RIGHT_ANGLE_VAL         732          /* 732   */

#define FACT_LEFT_MIN_ANGLE          400          /* 0     */ 
#define FACT_LEFT_MAX_ANGLE          2450         /* 180   */ 
#define FACT_LEFT_ANGLE_VAL          1636         /* 1636  */

#define FACT_TWIST_MIN_ANGLE         400          /* 0     */
#define FACT_TWIST_MAX_ANGLE         2450         /* 180   */
#define FACT_TWIST_ANGLE_VAL         1455         /* 1455  */

#define FACT_WRIST_MIN_ANGLE         400          /* 0     */ 
#define FACT_WRIST_MAX_ANGLE         2450         /* 180   */ 
#define FACT_WRIST_ANGLE_VAL         1428         /* 1428  */

#define FACT_FINGER_MIN_ANGLE        500          /* close */
#define FACT_FINGER_MAX_ANGLE        1560         /* open  */
#define FACT_FINGER_ANGLE_VAL        500          /* close */

/* === Addition Compute Parameters === */

#define CUBIC_SCALE_POS     1000
#define CUBIC_SCALE_A2      1000000
#define CUBIC_SCALE_A3      1000000000
#define CUBIC_COEFF_A2      3
#define CUBIC_COEFF_A3     -2
#define CUBIC_ZERO          0
#define ELAPSE_INIT         0

/* === Arm Control API === */

namespace ArmControl {
  void initialize();
  void handleSerialCommands(); void updateMotion();
}

#endif

