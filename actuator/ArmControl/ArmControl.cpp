#include "ArmControl.h"

using namespace ArmControl;

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
static SI_16 FactoryInit[MAX_JOINT] = {
  map(0, -109, 109, SERVO_PWM_MIN, SERVO_PWM_MAX),
  map(37, 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX),
  map(127, 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX),
  map(109, 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX),
  map(11, 0, 198, SERVO_PWM_MAX, SERVO_PWM_MIN),
  map(108, 0, 206, SERVO_PWM_MIN, SERVO_PWM_MAX)
};

/* === Store current joint values read from EEPROM === */
static SI_16 CurrentEEPROMValue[MAX_JOINT] = {{0}};

/* === Store user input joint data === */
static SI_16 UserInputJointData[MAX_JOINT] = {{0}};

/* === Servo driver object === */
static Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

/* === Joint channels mapping === */
static UI_8 JointChannels[MAX_CHANNEL] = {BASE_JOINT, RIGHT_JOINT, LEFT_JOINT, TWIST_JOINT, WRIST_JOINT, FINGER_JOINT};

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

/* === Motion State === */
static SI_16 previousJointPulse[MAX_JOINT] = {SERVO_PWM_MIN};
static SI_16 currentJointPulse[MAX_JOINT]  = {SERVO_PWM_MIN};
static SI_16 incomingJointPulse[MAX_JOINT] = {SERVO_PWM_MIN};
static UI_8 MotionDuration = MOTION_DURATION;                               /* in miliseconds */
static UI_8 jointMotionComplete[MAX_JOINT] = {D_FALSE};
/* === Queue Utilities === */
static UI_8 QueueIsFull(Trajectory *queue);
static UI_8 QueueIsEmpty(Trajectory *queue);
static void QueueEnqueue(Trajectory *queue, SI_16 *data);
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
static void WriteDataToEEPROM(SI_16 *value, UI_8 magic_val);
static void ReadJointDataFromEEPROM(SI_16 *value);
static void initializeEEPROM();

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

static void QueueEnqueue(Trajectory *queue, SI_16 *data) {
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

static UI_16 computeCubicTrajectory(UI_16 start, UI_16 end, SI_16 jointIdx) {
  static UI_16 startTime[MAX_JOINT];
  static UI_16 previousEnd[MAX_JOINT];
  static UI_16 outputValue[MAX_JOINT];
  float t;
  float duration;
  float a0, a1, a2, a3;

  t = (millis() - startTime[jointIdx]) / 1000.0;
  duration = MotionDuration / 1000.0;
  if (previousEnd[jointIdx] != end) {
    startTime[jointIdx] = millis();
    previousEnd[jointIdx] = end;
  }
  a0 = start;
  a1 = 0;
  a2 = 3.0 / (duration * duration) * (end - start);
  a3 = -2.0 / (duration * duration * duration) * (end - start);
  if (t <= duration) {
    outputValue[jointIdx] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  } else {
    jointMotionComplete[jointIdx] = D_TRUE;
  }

  return outputValue[jointIdx];
}

static void UpdateJointState() {
  UI_8 count;

  for (count = 0; count < MAX_JOINT; count++) {
    previousJointPulse[count] = CurrentEEPROMValue[count];
  }
  for (count = 0; count < MAX_JOINT; count++) {
    currentJointPulse[count] = previousJointPulse[count];
    servoDriver.writeMicroseconds(JointChannels[count], previousJointPulse[count]);
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
  SI_16 j1, j2, j3, j4, j5, j6, gripper;

  j1 = rawInput.substring(1, rawInput.indexOf('a')).toInt();
  j2 = rawInput.substring(rawInput.indexOf('a') + 1, rawInput.indexOf('b')).toInt();
  j3 = rawInput.substring(rawInput.indexOf('b') + 1, rawInput.indexOf('c')).toInt();
  j4 = rawInput.substring(rawInput.indexOf('c') + 1, rawInput.indexOf('d')).toInt();
  j5 = rawInput.substring(rawInput.indexOf('d') + 1, rawInput.indexOf('e')).toInt();
  j6 = rawInput.substring(rawInput.indexOf('e') + 1, rawInput.indexOf('f')).toInt();
  gripper = rawInput.substring(rawInput.indexOf('f') + 1).toInt();

  value[BASE]   = map(constrain(j1 + 109, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  value[RIGHT]  = map(constrain(172 - j2, 5, 140), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  value[LEFT]   = map(constrain(j3 + j2 + 37, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  value[TWIST]  = map(constrain(j4 + 109, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  value[WRIST]  = map(constrain(j5 + 101, 0, 198), 0, 198, SERVO_PWM_MAX, SERVO_PWM_MIN);
  value[FINGER] = map(constrain(j6 + 108, 0, 206), 0, 206, SERVO_PWM_MIN, SERVO_PWM_MAX);
  value[6] = gripper;

  QueueEnqueue(Queue, value);
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
static void WriteDataToEEPROM(SI_16 *value, UI_8 magic_val) {
  UI_8 count;

  for (count = 0; count < MAX_JOINT; count++) {
    EEPROM.write(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + MSB_OFFSET, (value[count] >> SHIFT_BYTE) & BYTE_MASK); /* MSB */ 
    EEPROM.write(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + LSB_OFFSET, value[count] & BYTE_MASK);                /* LSB */ 
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

/* === Exposed Methods === */

void ArmControl::initialize() {
  Serial.begin(SERCHNL);
  servoDriver.begin();
  servoDriver.setOscillatorFrequency(OCLFRQ);
  servoDriver.setPWMFreq(SERVO_FREQUENCY);
  delay(WAITINIT);

  initializeEEPROM();

  UpdateJointState();
}

void ArmControl::handleSerialCommands() {
  if (Serial.available() > 0) {
    String rawInput = Serial.readStringUntil('\n');
    if (rawInput.startsWith("s")) {
      HandleDataInput(currentJointPulse, &TrajectoryQueue, rawInput);
    } else if (rawInput == "init") {
      UpdateJointState();
    } else if (rawInput == "report") {
      reportCurrentJointPositions();
    } else if (rawInput == "mcalib") {
      ReadManualCalib(UserInputJointData);
    } else {
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
