#include "ArmControl.h"

using namespace ArmControl;

/* ================================================================= */
/* ===              Static variable decleration                  === */
/* ================================================================= */

/* === All joint channels and the PWM of the servos === */
enum {
  BASE = FIRST_JOINT,
  RIGHT,
  LEFT,
  TWIST,
  WRIST,
  FINGER,
  MAX_JOINT 
};

/* === Factory value for each servo of the arm === */
static const UI_16 FactoryInit[MAX_JOINT] = {
  FACT_BASE_ANGLE_VAL,
  FACT_RIGHT_ANGLE_VAL,
  FACT_LEFT_ANGLE_VAL,
  FACT_TWIST_ANGLE_VAL,
  FACT_WRIST_ANGLE_VAL,
  FACT_FINGER_ANGLE_VAL
};

/* === Store current joint values read from EEPROM === */
static UI_16 CurrentEEPROMValue[MAX_JOINT] = {{EMPTY_VALUE}};

/* === Store user input joint data === */
static UI_16 UserInputJointData[MAX_JOINT] = {{EMPTY_VALUE}};

/* === Servo driver object === */
static Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

/* === Joint channels mapping === */
static const UI_8 JointChannels[MAX_JOINT] = {BASE_JOINT, RIGHT_JOINT, LEFT_JOINT, TWIST_JOINT, WRIST_JOINT, FINGER_JOINT};

/* === Queue Structure === */
typedef struct {
  SI_8 front, rear, maxSize;
  UI_16 *qBase[MAX_JOINT];
} Trajectory;

/* === Buffer for trajectory queues === */
static UI_16 queueBuffer[MAX_JOINT][TRAJECTORY_QUEUE_SIZE] = {{EMPTY_VALUE}};

/* === Trajectory queue instance === */
static Trajectory TrajectoryQueue = {
  QUEUE_ERROR, QUEUE_ERROR, TRAJECTORY_QUEUE_SIZE,
  {queueBuffer[BASE], queueBuffer[RIGHT], queueBuffer[LEFT],
   queueBuffer[TWIST], queueBuffer[WRIST], queueBuffer[FINGER]}
};

static const UI_8 BaseAngle[] = {
  /*     Min                    Max      */
  BASE_MIN_ANGLE_PIN,      BASE_MAX_ANGLE_PIN
};

static const UI_8 RightAngle[] = {
  /*     Min                    Max      */
  RIGHT_MIN_ANGLE_PIN,      RIGHT_MAX_ANGLE_PIN
};

static const UI_8 LeftAngle[] = {
  /*     Min                    Max      */
  LEFT_MIN_ANGLE_PIN,      LEFT_MAX_ANGLE_PIN
};

static const UI_8 TwistAngle[] = {
  /*     Min                    Max      */
  TWIST_MIN_ANGLE_PIN,      TWIST_MAX_ANGLE_PIN
};

static const UI_8 WristAngle[] = {
  /*     Min                    Max      */
  WRIST_MIN_ANGLE_PIN,      WRIST_MAX_ANGLE_PIN
};

static const UI_8 FingerAngle[] = {
  /*     Min                    Max      */
  FINGER_MIN_ANGLE_PIN,      FINGER_MAX_ANGLE_PIN
};

static const UI_8 *PinTable[] = {
  BaseAngle,
  RightAngle,
  LeftAngle,
  TwistAngle,
  WristAngle,
  FingerAngle,
};

/* ================================================================= */
/* ===              Static function decleration                  === */
/* ================================================================= */

/* === Commond Function === */

static UI_16 AngletoValue(UI_16 angle, UI_16 minTarget, UI_16 maxTarget);
static UI_16 ConsTrain(UI_16 value);
static UI_16 ValuetoAngle(UI_16 value, UI_16 minTarget, UI_16 maxTarget);

/* === Motion State === */
static UI_16 previousJointPulse[MAX_JOINT] = {COMMON_MIN_ANGLE};
static UI_16 currentJointPulse[MAX_JOINT]  = {COMMON_MIN_ANGLE};
static UI_16 incomingJointPulse[MAX_JOINT] = {COMMON_MIN_ANGLE};
static UI_8 jointMotionComplete[MAX_JOINT] = {D_FALSE};

/* === Queue Utilities === */
static UI_8 QueueIsFull(Trajectory *queue);
static UI_8 QueueIsEmpty(Trajectory *queue);
static void QueueEndQueue(Trajectory *queue, UI_16 *data);
static void QueueDequeue(Trajectory *queue, UI_16 *dataOut);

/* === Servo and Motion State Utilities === */
static void testAllServos();
static UI_8 isAllJointMotionComplete();
static void prepareNextMotionStep();
static void updateServoPositions();
static UI_16 computeCubicTrajectory(UI_16 start, UI_16 end, UI_8 jointIdx);
static void UpdateJointState();
static void reportCurrentJointPositions();

/* === Input Handling === */
static void HandleDataInput(UI_16 *value, Trajectory *Queue, String rawInput);
static void ReadManualCalib(UI_16 *value);
static void AutoCalibration();
static UI_16 GetTravelPoint(UI_8 joint, UI_8 pin);

/* === EEPROM Utilities === */
static void WriteDataToEEPROM(const UI_16 *value, UI_8 magic_val);
static void ReadJointDataFromEEPROM(UI_16 *value);
static void initializeEEPROM();

/* === System input data === */

static void initializeTriggerPin();
static UI_8 GetTriggerData(UI_8 joint, UI_8 value);

/* === Command maping to function === */

struct CommandEntry {
    const char* name;
    CommandFunc func;
};

CommandEntry commands[] = {
    {"init",   UpdateJointState},
    {"report", reportCurrentJointPositions},
    {"mcalib", [](){ ReadManualCalib(UserInputJointData); }},
    {"acalib", AutoCalibration},
    {"test",   testAllServos}
};

/* ================================================================= */
/* ===              Static function defination                   === */
/* ================================================================= */

/* === Commond Function === */

static UI_16 AngletoValue(UI_16 angle, UI_16 minTarget, UI_16 maxTarget)
{
  UI_16 convertedAngle;
  UI_16 value;

  convertedAngle = ConsTrain(angle);
  value = (convertedAngle - SERVO_MIN_SOURCE) * (maxTarget - minTarget) / (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE) + minTarget;

  return value;
}

static UI_16 ConsTrain(UI_16 value)
{
  UI_16 ret;

  if (value < SERVO_MIN_SOURCE) {
    ret = SERVO_MIN_SOURCE;
  } else if (value > SERVO_MAX_SOURCE) {
    ret = SERVO_MAX_SOURCE;
  } else {
    ret = value;
  }

  return ret;
}

static UI_16 ValuetoAngle(UI_16 value, UI_16 minTarget, UI_16 maxTarget)
{
  UI_16 angle;

  angle = ((value - minTarget) * (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE)) / (maxTarget - minTarget) + SERVO_MIN_SOURCE;

  return angle;
}

/* === Queue Utilities === */
static UI_8 QueueIsFull(Trajectory *queue)
{
  UI_8 ret;

  ret = (queue->front == (queue->rear + INCREASE) % queue->maxSize) ? D_TRUE : D_FALSE;

  return ret;
}

static UI_8 QueueIsEmpty(Trajectory *queue)
{
  UI_8 ret;

  ret = (queue->front == QUEUE_ERROR) ? D_TRUE : D_FALSE;

  return ret;
}

static void QueueEndQueue(Trajectory *queue, UI_16 *data)
{
  UI_8 count;

  if ((D_FALSE == QueueIsFull(queue)) || 
      (D_TRUE == QueueIsEmpty(queue))) {
    queue->front = QUEUE_EMPTY;
    queue->rear = (queue->rear + INCREASE) % queue->maxSize;
    for (count = FIRST_JOINT; count < MAX_JOINT; count++ ) {
      queue->qBase[count][queue->rear] = data[count];
    }
  } else {
    /* do nothing */
  }
}

static void QueueDequeue(Trajectory *queue, UI_16 *dataOut)
{
  UI_8 count;

  if (D_FALSE == QueueIsEmpty(queue)) {
    for (count = FIRST_JOINT; count < MAX_JOINT; count++) {
      dataOut[count] = queue->qBase[count][queue->front];
    }
    if (queue->front == queue->rear) {
      queue->front = QUEUE_ERROR;
      queue->rear = QUEUE_ERROR;
    } else {
      queue->front = (queue->front + INCREASE) % queue->maxSize;
    }
  }
}

/* === Servo and Motion State Utilities === */
static void testAllServos()
{
  UI_8 count;
  UI_16 pulseMin;
  UI_16 pulseMid;

  pulseMin = COMMON_MIN_ANGLE;
  pulseMid = (COMMON_MIN_ANGLE + COMMON_MAX_ANGLE) / MED_DEVIDE;

  for (count = FIRST_JOINT; count < MAX_JOINT; count++) {
    if (FINGER == count) {
      servoDriver.writeMicroseconds(JointChannels[count], FACT_FINGER_MIN_ANGLE);
      delay(DELAY_TIME);
      servoDriver.writeMicroseconds(JointChannels[count], FACT_FINGER_MAX_ANGLE);
      delay(DELAY_TIME);
    } else {
      servoDriver.writeMicroseconds(JointChannels[count], pulseMin);
      delay(DELAY_TIME);
      servoDriver.writeMicroseconds(JointChannels[count], pulseMid);
      delay(DELAY_TIME);
  }
  }
}

static UI_8 isAllJointMotionComplete()
{
  UI_8 count;
  UI_8 ret;

  ret = D_TRUE;
  for (count = FIRST_JOINT; count < MAX_JOINT; count++) {
    if (D_FALSE == jointMotionComplete[count]) {
      ret = D_FALSE;
      break;
    }
  }

  return ret;
}

static void prepareNextMotionStep()
{
  UI_8 count;

  for (count = FIRST_JOINT; count < MAX_JOINT; count++) {
    previousJointPulse[count] = currentJointPulse[count];
    jointMotionComplete[count] = D_FALSE;
  }
  if (D_FALSE == QueueIsEmpty(&TrajectoryQueue)) {
    QueueDequeue(&TrajectoryQueue, currentJointPulse);
  }
}

static void updateServoPositions()
{
  UI_8 count;

  for (count = FIRST_JOINT; count < MAX_JOINT; count++) {
    servoDriver.writeMicroseconds(JointChannels[count], computeCubicTrajectory(previousJointPulse[count], currentJointPulse[count], count));
  }
}


static UI_16 computeCubicTrajectory(UI_16 startPos, UI_16 endPos, UI_8 jointIdx)
{
  static UI_16 startTime[MAX_JOINT];
  static UI_16 lastTarget[MAX_JOINT];
  static UI_16 currentOutput[MAX_JOINT];
  SI_32 a0;
  SI_32 a1;
  SI_32 a2;
  SI_32 a3;
  UI_16 elapsedMs;
  SI_32 T;
  SI_32 t;
  SI_32 delta;
  SI_32 pos;

  delta = (SI_32)endPos - (SI_32)startPos;

  if (lastTarget[jointIdx] != endPos) {
    startTime[jointIdx] = millis();
    lastTarget[jointIdx] = endPos;
    elapsedMs = ELAPSE_INIT;
  }

  elapsedMs = millis() - startTime[jointIdx];
  T = MOTION_DURATION;          // total duration in ms

  t = elapsedMs;
 
  // Precompute coefficients scaled by 1e9 to maintain precision
  a0 = startPos * CUBIC_SCALE_POS;
  a1 = CUBIC_ZERO;
  a2 = (CUBIC_COEFF_A2 * delta * CUBIC_SCALE_A2) / (T * T);
  a3 = (CUBIC_COEFF_A3 * delta * CUBIC_SCALE_A3) / (T * T * T);
     

  if (t <= T) {
    // Compute position using scaled math (divide appropriately)
    pos = a0 / CUBIC_SCALE_POS + (a1 * t) 
             + (a2 * t * t) / CUBIC_SCALE_A2
             + (a3 * t * t * t) / CUBIC_SCALE_A3;
    currentOutput[jointIdx] = (UI_16)pos;
  } else {
    jointMotionComplete[jointIdx] = D_TRUE;
  }

  return currentOutput[jointIdx];
}

static void UpdateJointState()
{
  UI_8 count;

  for (count = FIRST_JOINT; count < MAX_JOINT; count++) {
    previousJointPulse[count] = CurrentEEPROMValue[count];
  }
  for (count = FIRST_JOINT; count < MAX_JOINT; count++) {
    currentJointPulse[count] = previousJointPulse[count];
    servoDriver.writeMicroseconds(JointChannels[count], currentJointPulse[count]);
  }
}

static void reportCurrentJointPositions()
{
  UI_8 count;
  UI_16 val;
  UI_16 pulseMin;
  UI_16 pulseMax;

  pulseMin = COMMON_MIN_ANGLE;
  pulseMax = COMMON_MAX_ANGLE;

  Serial.print("Current joint pulses: ");
  for (count = FIRST_JOINT; count < MAX_JOINT; count++) {
    if (FINGER == count)
    {
      pulseMin = FACT_FINGER_MIN_ANGLE;
      pulseMax = FACT_FINGER_MAX_ANGLE;
    }
    val = ValuetoAngle(currentJointPulse[count], pulseMin, pulseMax);
    Serial.print(val);
    Serial.print(count < FINGER ? ", " : "\n");
  }
}

/* === Input Handling === */
static void HandleDataInput(UI_16 *value, Trajectory *Queue, String rawInput)
{
  UI_16 base, right, left, twist, wrist, finger;

  base    = rawInput.substring(FIRST_INPUT, rawInput.indexOf('l')).toInt();
  right   = rawInput.substring(rawInput.indexOf('l') + INCREASE, rawInput.indexOf('r')).toInt();
  left    = rawInput.substring(rawInput.indexOf('r') + INCREASE, rawInput.indexOf('t')).toInt();
  twist   = rawInput.substring(rawInput.indexOf('t') + INCREASE, rawInput.indexOf('w')).toInt();
  wrist   = rawInput.substring(rawInput.indexOf('w') + INCREASE, rawInput.indexOf('f')).toInt();
  finger  = rawInput.substring(rawInput.indexOf('f') + INCREASE).toInt();

  value[BASE]        = AngletoValue(base, FACT_BASE_MIN_ANGLE, FACT_BASE_MAX_ANGLE);
  value[RIGHT]       = AngletoValue(right, FACT_RIGHT_MIN_ANGLE, FACT_RIGHT_MAX_ANGLE);
  value[LEFT]        = AngletoValue(left, FACT_LEFT_MIN_ANGLE, FACT_LEFT_MAX_ANGLE);
  value[TWIST]       = AngletoValue(twist, FACT_TWIST_MIN_ANGLE, FACT_TWIST_MAX_ANGLE);
  value[WRIST]       = AngletoValue(wrist, FACT_WRIST_MIN_ANGLE, FACT_WRIST_MAX_ANGLE);
  value[FINGER]      = AngletoValue(finger, FACT_FINGER_MIN_ANGLE, FACT_FINGER_MAX_ANGLE);

  QueueEndQueue(Queue, value);
}

static void ReadManualCalib(UI_16 *value)
{
  UI_8 count;
  UI_8 low, high;

  for (count = FIRST_JOINT; count < MAX_JOINT; count++) {
    low = Serial.read();
    high = Serial.read();
    value[count] = (high << SHIFT_BYTE) | low;
  }
  WriteDataToEEPROM(value, MAGIC_MODF_VAL);
  ReadJointDataFromEEPROM(CurrentEEPROMValue);
}

/* === EEPROM Utilities === */
static void WriteDataToEEPROM(const UI_16 *value, UI_8 magic_val)
{
  UI_8 count;
  
  for (count = FIRST_JOINT; count < MAX_JOINT; count++) {
    /* The upper 8 bits of a 16-bit value */
    EEPROM.write(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + MSB_OFFSET, (value[count] >> SHIFT_BYTE) & BYTE_MASK); /* MSB */ 
    /* The lower 8 bits of a 16-bit value */
    EEPROM.write(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + LSB_OFFSET, value[count] & BYTE_MASK);                 /* LSB */ 
  }
  
  EEPROM.write(MAGIC_ADDR, magic_val);

  EEPROM.commit();
}

static void ReadJointDataFromEEPROM(UI_16 *value)
{
  UI_8 count;
  UI_8 msb, lsb;

  for (count = FIRST_JOINT; count < MAX_JOINT; count++) {
    msb = EEPROM.read(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + MSB_OFFSET);
    lsb = EEPROM.read(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + LSB_OFFSET);
    value[count] = ((msb << SHIFT_BYTE) | lsb);
  }
}

static void initializeEEPROM()
{
  UI_8 eepVal;

  if (!EEPROM.begin(EEPROM_SIZE)) {
    return;
  }

  eepVal = EEPROM.read(MAGIC_ADDR);
  if ((eepVal != MAGIC_INIT_VAL) && (eepVal != MAGIC_MODF_VAL)) {
    WriteDataToEEPROM(FactoryInit, MAGIC_INIT_VAL);
  } else {
    /* do nothing */
  }
  ReadJointDataFromEEPROM(CurrentEEPROMValue);
}

/* === System input data === */

static void initializeTriggerPin()
{
  UI_8 joint;
  UI_8 index;

  for (joint = FIRST_JOINT; joint < MAX_JOINT; joint++) {
    for (index = FIRST_ANGLE;index < MAX_ANGLE; index++) {
      pinMode(PinTable[joint][index],INPUT);
    }
  }
}

static UI_8 GetTriggerData(UI_8 joint, UI_8 index)
{
  UI_8 ret;

  ret = D_OFF;
  ret = digitalRead(PinTable[joint][index]);

  return ret;
}

static void AutoCalibration()
{
  UI_8 joint;
  UI_8 index;
  UI_16 ServoAngleBuffer[joint][index];

  for (joint = FIRST_JOINT; joint < MAX_JOINT; joint++) {
    for (index = FIRST_ANGLE; index < MAX_ANGLE; index++) {
      ServoAngleBuffer[joint][index] = GetTravelPoint(joint, index);
    }
  }
}

static UI_16 GetTravelPoint(UI_8 joint, UI_8 pin)
{
  UI_16 start;
  UI_16 end;
  UI_8 step;
  UI_16 ret;
  UI_8 angle;
  UI_8 triggerData;

  triggerData = GetTriggerData(joint, pin);
  start = (SERVO_MIN_TARGET + SERVO_MAX_TARGET)/MED_DEVIDE;
  angle = start;
  pinMode(PinTable[joint][pin], INPUT);

  if (PIN_MIN == pin) {
    end = SERVO_MIN_TARGET;
    step = -STEP_SIZE;
  } else if (PIN_MAX == pin) {
    end = SERVO_MAX_TARGET;
    step = STEP_SIZE;
  } else {
    end = start;
  }

  for (;(pin ? angle <= end : angle >= end); angle += step) {
      servoDriver.writeMicroseconds(JointChannels[joint], angle);
      delay(DELAY_SET);
      if (D_ON == triggerData) {
        ret = angle;
      } else {
        ret = pin ? start : end;
      }
  }

  return ret;
}

/* ================================================================= */
/* ===                  Exposed Methods                          === */
/* ================================================================= */

void ArmControl::initialize()
{
  Serial.begin(SERCHNL);
  servoDriver.begin();
  servoDriver.setOscillatorFrequency(OCLFRQ);
  servoDriver.setPWMFreq(SERVO_FREQUENCY);
  delay(WAITINIT);

  initializeEEPROM();
  initializeTriggerPin();

  UpdateJointState();
}

void ArmControl::handleSerialCommands()
{
  UI_8 count;

  if (Serial.available() > UNAVAIL_SER) {
    String rawInput = Serial.readStringUntil('\n');

    if (rawInput.startsWith("b")) {
      HandleDataInput( incomingJointPulse, &TrajectoryQueue, rawInput );
    } else {
      for (count = INIT_COMMAND ; count < (sizeof(commands)/sizeof(commands[INIT_COMMAND])); ++count) {
        if (rawInput == commands[count].name) {
          commands[count].func();
          break;
        }
      }
    }
  }
}

void ArmControl::updateMotion()
{
  updateServoPositions();
  if (D_TRUE == isAllJointMotionComplete()) {
    prepareNextMotionStep();
  }
}
