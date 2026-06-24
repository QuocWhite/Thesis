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


/* === Servo Parameters === */
#define SERVO_MIN_TARGET        320
#define SERVO_MAX_TARGET        2570
#define SERVO_FREQUENCY         50
#define MOTION_DURATION         150
#define MIDPOINT_DIV            2
#define DELAY_TIME              500

#define SERVO_MIN_SOURCE        0
#define SERVO_MAX_SOURCE        180
#define EMPTY_VALUE             0

/* === Queue Parameters === */
#define TRAJECTORY_QUEUE_SIZE   16
#define QUEUE_NULL_IDX          -1
#define INCREMENT               1

#define QUEUE_EMPTY             0
#define SERIAL_AVAIL_MIN        0

/* === Matrix/Array Dimensions === */
#define MAX_ANGLE       2
#define FIRST_JOINT     0
#define FIRST_ANGLE     0

/* === EEPROM Configuration === */
#define EEPROM_SIZE       32
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
#define COMMON_MIN_ANGLE             400
#define COMMON_MAX_ANGLE             2450

#define FACT_BASE_MIN_ANGLE          400
#define FACT_BASE_MAX_ANGLE          2450
#define FACT_BASE_ANGLE_VAL          1455

#define FACT_RIGHT_MIN_ANGLE         400
#define FACT_RIGHT_MAX_ANGLE         2450
#define FACT_RIGHT_ANGLE_VAL         732

#define FACT_LEFT_MIN_ANGLE          400
#define FACT_LEFT_MAX_ANGLE          2450
#define FACT_LEFT_ANGLE_VAL          1636

#define FACT_TWIST_MIN_ANGLE         400
#define FACT_TWIST_MAX_ANGLE         2450
#define FACT_TWIST_ANGLE_VAL         1455

#define FACT_WRIST_MIN_ANGLE         400
#define FACT_WRIST_MAX_ANGLE         2450
#define FACT_WRIST_ANGLE_VAL         1428

#define FACT_FINGER_MIN_ANGLE        500
#define FACT_FINGER_MAX_ANGLE        1560
#define FACT_FINGER_ANGLE_VAL        500

/* === Addition Compute Parameters === */
#define CUBIC_SCALE_POS     1000
#define CUBIC_SCALE_A2      1000000
#define CUBIC_SCALE_A3      1000000000
#define CUBIC_COEFF_A2      3
#define CUBIC_COEFF_A3     -2
#define CUBIC_ZERO          0


#define INIT_COMMAND     0

/* === Arm Control OOP API === */
class ArmControl {
public:
  ArmControl();

  void initialize();
  void handleSerialCommands();
  void updateMotion();

private:
  /* === Joint Index === */
  enum JointIdx : UI_8 {
    BASE = FIRST_JOINT,
    RIGHT,
    LEFT,
    TWIST,
    WRIST,
    FINGER,
    MAX_JOINT
  };

  /* === Queue === */
  struct Trajectory {
    SI_8 front;
    SI_8 rear;
    SI_8 maxSize;
    UI_16* qBase[MAX_JOINT];
  };

  /* === Command dispatch (member function pointer) === */
  typedef void (ArmControl::*CommandMethod)();

  struct CommandEntry {
    const char* name;
    CommandMethod method;
  };

  /* === Owned objects/state === */
  Adafruit_PWMServoDriver servoDriver_;

  UI_16 FactoryInit_[MAX_JOINT];
  UI_16 CurrentEEPROMValue_[MAX_JOINT];
  UI_16 UserInputJointData_[MAX_JOINT];

  UI_8 JointChannels_[MAX_JOINT];

  UI_16 queueBuffer_[MAX_JOINT][TRAJECTORY_QUEUE_SIZE];
  Trajectory TrajectoryQueue_;

  UI_16 previousJointPulse_[MAX_JOINT];
  UI_16 currentJointPulse_[MAX_JOINT];
  UI_16 incomingJointPulse_[MAX_JOINT];
  UI_8  jointMotionComplete_[MAX_JOINT];

  /* Cubic trajectory state (was static inside computeCubicTrajectory) */
  UI_16 startTime_[MAX_JOINT];
  UI_16 lastTarget_[MAX_JOINT];
  UI_16 currentOutput_[MAX_JOINT];

  /* === Pin table === */
  UI_8 BaseAngle_[MAX_ANGLE];
  UI_8 RightAngle_[MAX_ANGLE];
  UI_8 LeftAngle_[MAX_ANGLE];
  UI_8 TwistAngle_[MAX_ANGLE];
  UI_8 WristAngle_[MAX_ANGLE];
  UI_8 FingerAngle_[MAX_ANGLE];
  const UI_8* PinTable_[MAX_JOINT];

  /* === Commands table === */
  static const CommandEntry commands_[];
  static const UI_8 commandCount_;

  /* === Helpers (pure conversions) === */
  UI_16 angleToValue_(UI_16 angle, UI_16 minTarget, UI_16 maxTarget);
  UI_16 constrain_(UI_16 value);
  UI_16 valueToAngle_(UI_16 value, UI_16 minTarget, UI_16 maxTarget);

  /* === Queue helpers === */
  UI_8 queueIsFull_(Trajectory* queue);
  UI_8 queueIsEmpty_(Trajectory* queue);
  void enqueue_(Trajectory* queue, UI_16* data);
  void dequeue_(Trajectory* queue, UI_16* dataOut);

  /* === Motion helpers === */
  void testAllServos_();
  UI_8 isAllJointMotionComplete_();
  void prepareNextMotionStep_();
  void updateServoPositions_();
  UI_16 computeCubicTrajectory_(UI_16 startPos, UI_16 endPos, UI_8 jointIdx);
  void updateJointState_();
  void reportCurrentJointPositions_();

  /* === Input handling === */
  void handleDataInput_(UI_16* value, Trajectory* Queue, String rawInput);
  void readManualCalib_(UI_16* value);
  void autoCalibration_();
  UI_16 getTravelPoint_(UI_8 joint, UI_8 pin);

  /* === EEPROM utilities === */
  void writeDataToEEPROM_(const UI_16* value, UI_8 magic_val);
  void readJointDataFromEEPROM_(UI_16* value);
  void initializeEEPROM_();

  /* === Trigger input === */
  void initializeTriggerPin_();
  UI_8 getTriggerData_(UI_8 joint, UI_8 index);

  /* === Command wrappers (so command table stays simple) === */
  void cmdInit_();
  void cmdReport_();
  void cmdManualCalib_();
  void cmdAutoCalib_();
  void cmdTest_();
};


#endif
