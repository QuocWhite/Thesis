#include "ArmControl.h"

using namespace ArmControl;

/* ================================================================= */
/* ===              Static variable decleration                  === */
/* ================================================================= */

/* === All joint channels and the PWM of the servos === */
enum {
  BASE = 0,
  RIGHT,
  LEFT,
  TWIST,
  WRIST,
  FINGER,
  SERVO_PWM,
  MAX_CHANNEL 
};

/* === Factory value for each servo of the arm === */
static const SI_16 FactoryInit[MAX_JOINT] = {
  map(BASE_ANGLE_VAL, BASE_MIN_ANGLE, BASE_MAX_ANGLE, SERVO_PWM_MIN, SERVO_PWM_MAX),
  map(RIGHT_ANGLE_VAL, RIGHT_MIN_ANGLE, RIGHT_MAX_ANGLE, SERVO_PWM_MIN, SERVO_PWM_MAX),
  map(LEFT_ANGLE_VAL, LEFT_MIN_ANGLE, LEFT_MAX_ANGLE, SERVO_PWM_MIN, SERVO_PWM_MAX),
  map(TWIST_ANGLE_VAL, TWIST_MIN_ANGLE, TWIST_MAX_ANGLE, SERVO_PWM_MIN, SERVO_PWM_MAX),
  map(WRIST_ANGLE_VAL, WRIST_MIN_ANGLE, WRIST_MAX_ANGLE, SERVO_PWM_MAX, SERVO_PWM_MIN),
  map(FINGER_ANGLE_VAL, FINGER_MIN_ANGLE, FINGER_MAX_ANGLE, SERVO_PWM_MIN, SERVO_PWM_MAX)
};

/* === Store current joint values read from EEPROM === */
static SI_16 CurrentEEPROMValue[MAX_JOINT] = {{0}};

/* === Store user input joint data === */
static SI_16 UserInputJointData[MAX_JOINT] = {{0}};

/* === Servo driver object === */
static Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

/* === Joint channels mapping === */
static const UI_8 JointChannels[MAX_JOINT] = {BASE_JOINT, RIGHT_JOINT, LEFT_JOINT, TWIST_JOINT, WRIST_JOINT, FINGER_JOINT};

/* === Queue Structure === */
typedef struct {
  SI_8 front, rear, maxSize;
  UI_16 *qBase[MAX_CHANNEL];
} Trajectory;

/* === Buffer for trajectory queues === */
static UI_16 queueBuffer[MAX_CHANNEL][TRAJECTORY_QUEUE_SIZE] = {{0}};

/* === Trajectory queue instance === */
static Trajectory TrajectoryQueue = {
  QUEUE_ERROR, QUEUE_ERROR, TRAJECTORY_QUEUE_SIZE,
  {queueBuffer[BASE], queueBuffer[RIGHT], queueBuffer[LEFT],
   queueBuffer[TWIST], queueBuffer[WRIST], queueBuffer[FINGER], queueBuffer[MAX_CHANNEL]}
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

static const UI_8 *RangeAngleTable[] = {
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

/* === Motion State === */
static SI_16 previousJointPulse[MAX_JOINT] = {SERVO_PWM_MIN};
static SI_16 currentJointPulse[MAX_JOINT]  = {SERVO_PWM_MIN};
static SI_16 incomingJointPulse[MAX_JOINT] = {SERVO_PWM_MIN};
static UI_8 MotionDuration = MOTION_DURATION;                               /* in miliseconds */
static UI_8 jointMotionComplete[MAX_JOINT] = {D_FALSE};

/* === Queue Utilities === */
static UI_8 QueueIsFull(Trajectory *queue);
static UI_8 QueueIsEmpty(Trajectory *queue);
static void QueueEndQueue(Trajectory *queue, SI_16 *data);
static void QueueDequeue(Trajectory *queue, SI_16 *dataOut);

/* === Servo and Motion State Utilities === */
static void testAllServos();
static UI_8 isAllJointMotionComplete();
static void prepareNextMotionStep();
static void updateServoPositions();
static UI_16 computeCubicTrajectory(UI_16 start, UI_16 end, SI_16 jointIdx);
static void UpdateJointState();
static void reportCurrentJointPositions();

/* === Input Handling === */
static void HandleDataInput(SI_16 *value, Trajectory *Queue, String rawInput);
static void ReadManualCalib(SI_16 *value);

/* === EEPROM Utilities === */
static void WriteDataToEEPROM(const SI_16 *value, UI_8 magic_val);
static void ReadJointDataFromEEPROM(SI_16 *value);
static void initializeEEPROM();

/* === System input data === */

static void initializeTriggerPin();
static UI_8 GetTriggerData(UI_8 joint, UI_8 value);

/* ================================================================= */
/* ===              Static function defination                   === */
/* ================================================================= */

/* === Queue Utilities === */
static UI_8 QueueIsFull(Trajectory *queue) {
  UI_8 ret;

  ret = (queue->front == (queue->rear + INCREASE) % queue->maxSize) ? D_TRUE : D_FALSE;

  return ret;
}

static UI_8 QueueIsEmpty(Trajectory *queue) {
  UI_8 ret;

  ret = (queue->front == QUEUE_ERROR) ? D_TRUE : D_FALSE;

  return ret;
}

static void QueueEndQueue(Trajectory *queue, SI_16 *data) {
  UI_8 count;

  if ((D_FALSE == QueueIsFull(queue)) || 
      (D_TRUE == QueueIsEmpty(queue))) {
    queue->front = QUEUE_EMPTY;
    queue->rear = (queue->rear + INCREASE) % queue->maxSize;
    for (count = 0; count < MAX_CHANNEL; count++ ) {
      queue->qBase[count][queue->rear] = data[count];
    }
  } else {
    /* do nothing */
  }
}

static void QueueDequeue(Trajectory *queue, SI_16 *dataOut) {
  UI_8 count;

  if (D_FALSE == QueueIsEmpty(queue)) {
    for (count = 0; count < MAX_CHANNEL; count++) {
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
static void testAllServos() {
  UI_8 count;
  UI_16 pulseMin;
  UI_16 pulseMid;

  for (count = 0; count < MAX_JOINT; count++) {
    pulseMin = SERVO_PWM_MIN;
    pulseMid = (SERVO_PWM_MIN + SERVO_PWM_MAX) / MED_DEVIDE;
    servoDriver.writeMicroseconds(JointChannels[count], pulseMin);
    delay(DELAY_TIME);
    servoDriver.writeMicroseconds(JointChannels[count], pulseMid);
    delay(DELAY_TIME);
  }
}

static UI_8 isAllJointMotionComplete() {
  UI_8 count;
  UI_8 ret;

  ret = D_TRUE;
  for (count = 0; count < MAX_JOINT; count++) {
    if (D_FALSE == jointMotionComplete[count]) {
      ret = D_FALSE;
      break;
    }
  }

  return ret;
}

static void prepareNextMotionStep() {
  UI_8 count;

  for (count = 0; count < MAX_JOINT; count++) {
    previousJointPulse[count] = currentJointPulse[count];
    jointMotionComplete[count] = D_FALSE;
  }
  if (D_FALSE == QueueIsEmpty(&TrajectoryQueue)) {
    QueueDequeue(&TrajectoryQueue, currentJointPulse);
  }
}

static void updateServoPositions() {
  UI_8 count;

  for (count = 0; count < MAX_JOINT; count++) {
    servoDriver.writeMicroseconds(JointChannels[count], computeCubicTrajectory(previousJointPulse[count], currentJointPulse[count], count));
  }
}


static UI_16 computeCubicTrajectory(UI_16 startPos, UI_16 endPos, SI_16 jointIdx) {
  static UI_16 startTime[MAX_JOINT];
  static UI_16 lastTarget[MAX_JOINT];
  static UI_16 currentOutput[MAX_JOINT];

  // Get elapsed time in milliseconds
  UI_16 elapsedMs = millis() - startTime[jointIdx];
  UI_16 totalMs = MotionDuration;

  // If new target detected, reset motion timer
  if (lastTarget[jointIdx] != endPos) {
    startTime[jointIdx] = millis();
    lastTarget[jointIdx] = endPos;
    elapsedMs = 0;
  }

  // To keep precision, we’ll use scaled integer math (×1000)
  // Convert to scaled "seconds" units
  SI_32 t = elapsedMs;        // time in ms
  SI_32 T = totalMs;          // total duration in ms
  SI_32 delta = (SI_32)endPos - (SI_32)startPos;

  // Precompute coefficients scaled by 1e9 to maintain precision
  // a2 = 3 * (end - start) / (T^2)
  // a3 = -2 * (end - start) / (T^3)
  SI_32 a0 = startPos * 1000L;
  SI_32 a1 = 0;
  SI_32 a2 = (3L * delta * 1000000L) / (T * T);     // scaled by 1e6
  SI_32 a3 = (-2L * delta * 1000000000L) / (T * T * T); // scaled by 1e9

  if (t <= T) {
    // Compute position using scaled math (divide appropriately)
    SI_32 pos = a0
              + (a1 * t) / 1000L
              + (a2 * t * t) / 1000000L
              + (a3 * t * t * t) / 1000000000L;

    currentOutput[jointIdx] = (UI_16)(pos / 1000L);
  } else {
    jointMotionComplete[jointIdx] = D_TRUE;
  }

  return currentOutput[jointIdx];
}

static void UpdateJointState() {
  UI_8 count;

  for (count = 0; count < MAX_JOINT; count++) {
    previousJointPulse[count] = CurrentEEPROMValue[count];
  }
  for (count = 0; count < MAX_JOINT; count++) {
    currentJointPulse[count] = previousJointPulse[count];
    servoDriver.writeMicroseconds(JointChannels[count], currentJointPulse[count]);
  }
  currentJointPulse[SERVO_PWM] = MOTION_DURATION;
}

static void reportCurrentJointPositions() {
  UI_8 count;

  Serial.print("Current joint pulses: ");
  for (count = 0; count < MAX_JOINT; count++) {
    Serial.print(currentJointPulse[count]);
    Serial.print(count < FINGER ? ", " : "\n");
  }
  Serial.print("Gripper: ");
  Serial.println(currentJointPulse[SERVO_PWM]);
}

/* === Input Handling === */
static void HandleDataInput(SI_16 *value, Trajectory *Queue, String rawInput) {
  SI_16 base, right, left, twist, wrist, finger, gripper;

  base    = rawInput.substring(1, rawInput.indexOf('b')).toInt();
  right   = rawInput.substring(rawInput.indexOf('b') + INCREASE, rawInput.indexOf('r')).toInt();
  left    = rawInput.substring(rawInput.indexOf('r') + INCREASE, rawInput.indexOf('l')).toInt();
  twist   = rawInput.substring(rawInput.indexOf('l') + INCREASE, rawInput.indexOf('t')).toInt();
  wrist   = rawInput.substring(rawInput.indexOf('t') + INCREASE, rawInput.indexOf('w')).toInt();
  finger  = rawInput.substring(rawInput.indexOf('w') + INCREASE, rawInput.indexOf('f')).toInt();
  gripper = rawInput.substring(rawInput.indexOf('f') + INCREASE).toInt();

  value[BASE]        = map(constrain(base + 109, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  value[RIGHT]       = map(constrain(172 - right, 5, 140), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  value[LEFT]        = map(constrain(right + left + 37, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  value[TWIST]       = map(constrain(twist + 109, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  value[WRIST]       = map(constrain(wrist + 101, 0, 198), 0, 198, SERVO_PWM_MAX, SERVO_PWM_MIN);
  value[FINGER]      = map(constrain(finger + 108, 0, 206), 0, 206, SERVO_PWM_MIN, SERVO_PWM_MAX);
  value[SERVO_PWM]   = gripper;

  QueueEndQueue(Queue, value);
}

static void ReadManualCalib(SI_16 *value) {
  UI_8 count;
  UI_8 low, high;

  for (count = 0; count < MAX_JOINT; count++) {
    low = Serial.read();
    high = Serial.read();
    value[count] = (high << SHIFT_BYTE) | low;
  }
  WriteDataToEEPROM(value, MAGIC_MODF_VAL);
  ReadJointDataFromEEPROM(CurrentEEPROMValue);
}

/* === EEPROM Utilities === */
static void WriteDataToEEPROM(const SI_16 *value, UI_8 magic_val) {
  UI_8 count;

  for (count = 0; count < MAX_JOINT; count++) {
    /* The upper 8 bits of a 16-bit value */
    EEPROM.write(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + MSB_OFFSET, (value[count] >> SHIFT_BYTE) & BYTE_MASK); /* MSB */ 
    /* The lower 8 bits of a 16-bit value */
    EEPROM.write(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + LSB_OFFSET, value[count] & BYTE_MASK);                 /* LSB */ 
  }
  EEPROM.write(MAGIC_ADDR, magic_val);
  EEPROM.commit();
}

static void ReadJointDataFromEEPROM(SI_16 *value) {
  UI_8 count;
  UI_8 msb, lsb;

  for (count = 0; count < MAX_JOINT; count++) {
    msb = EEPROM.read(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + MSB_OFFSET);
    lsb = EEPROM.read(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + LSB_OFFSET);
    value[count] = ((msb << SHIFT_BYTE) | lsb);
  }
}

static void initializeEEPROM() {
  UI_8 eepVal;

  eepVal = EEPROM.read(MAGIC_ADDR);
  if (!EEPROM.begin(EEPROM_SIZE)) {
    return;
  }
  if (eepVal != MAGIC_INIT_VAL && eepVal != MAGIC_MODF_VAL) {
    WriteDataToEEPROM(FactoryInit, MAGIC_INIT_VAL);
  } else {
    /* do nothing */
  }
  ReadJointDataFromEEPROM(CurrentEEPROMValue);
}

/* === System input data === */

static void initializeTriggerPin() {
  UI_8 joint;
  UI_8 index;

  for (joint = 0; joint < MAX_JOINT; joint++) {
    for (index = 0;index < MAX_ANGLE; index++) {
      pinMode(RangeAngleTable[joint][index],OUTPUT);
    }
  }
}

static UI_8 GetTriggerData(UI_8 joint, UI_8 index) {
  UI_8 ret;
  UI_8 AnglePin;

  ret = D_OFF;
  AnglePin = digitalRead(RangeAngleTable[joint][index]);

  if (AnglePin == HIGH) {
    ret = D_ON;
  }

  return ret;
}

/* ================================================================= */
/* ===                  Exposed Methods                          === */
/* ================================================================= */

void ArmControl::initialize() {
  Serial.begin(SERCHNL);
  servoDriver.begin();
  servoDriver.setOscillatorFrequency(OCLFRQ);
  servoDriver.setPWMFreq(SERVO_FREQUENCY);
  delay(WAITINIT);

  initializeEEPROM();
  initializeTriggerPin();

  UpdateJointState();
}

void ArmControl::handleSerialCommands() {
  if (Serial.available() > 0) {
    String rawInput = Serial.readStringUntil('\n');
    if (rawInput.startsWith("s")) {
      UI_16 base, right, left, twist, wrist, finger, gripper;

      base    = rawInput.substring(1, rawInput.indexOf('b')).toInt();
      right   = rawInput.substring(rawInput.indexOf('b') + INCREASE, rawInput.indexOf('r')).toInt();
      left    = rawInput.substring(rawInput.indexOf('r') + INCREASE, rawInput.indexOf('l')).toInt();
      twist   = rawInput.substring(rawInput.indexOf('l') + INCREASE, rawInput.indexOf('t')).toInt();
      wrist   = rawInput.substring(rawInput.indexOf('t') + INCREASE, rawInput.indexOf('w')).toInt();
      finger  = rawInput.substring(rawInput.indexOf('w') + INCREASE, rawInput.indexOf('f')).toInt();
      gripper = rawInput.substring(rawInput.indexOf('f') + INCREASE).toInt();

      incomingJointPulse[BASE]        = map(constrain(base + 109, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[RIGHT]       = map(constrain(172 - right, 5, 140), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[LEFT]        = map(constrain(right + left + 37, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[TWIST]       = map(constrain(twist + 109, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[WRIST]       = map(constrain(wrist + 101, 0, 198), 0, 198, SERVO_PWM_MAX, SERVO_PWM_MIN);
      incomingJointPulse[FINGER]      = map(constrain(finger + 108, 0, 206), 0, 206, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[SERVO_PWM]   = gripper;

      QueueEndQueue(&TrajectoryQueue, incomingJointPulse);  
      /*HandleDataInput( incomingJointPulse, &TrajectoryQueue, rawInput );*/
    } else if (rawInput == "init") {
      UpdateJointState();
    } else if (rawInput == "report") {
      reportCurrentJointPositions();
    } else if (rawInput == "mcalib") {
      ReadManualCalib(UserInputJointData);
    } else if (rawInput == "test") {
      testAllServos();
    }else {
      /* do nothing */
    }
  }
}

void ArmControl::updateMotion() {
  updateServoPositions();
  if (D_TRUE == isAllJointMotionComplete()) {
    prepareNextMotionStep();
  }
}
