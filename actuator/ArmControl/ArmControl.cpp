#include "ArmControl.h"

/* ================================================================= */
/* ===                      Constructor                          === */
/* ================================================================= */

ArmControl::ArmControl()
: servoDriver_()
{
  /* === FactoryInit_ === */
  FactoryInit_[BASE]   = FACT_BASE_ANGLE_VAL;
  FactoryInit_[RIGHT]  = FACT_RIGHT_ANGLE_VAL;
  FactoryInit_[LEFT]   = FACT_LEFT_ANGLE_VAL;
  FactoryInit_[TWIST]  = FACT_TWIST_ANGLE_VAL;
  FactoryInit_[WRIST]  = FACT_WRIST_ANGLE_VAL;
  FactoryInit_[FINGER] = FACT_FINGER_ANGLE_VAL;

  /* === JointChannels_ === */
  JointChannels_[BASE]   = BASE_JOINT;
  JointChannels_[RIGHT]  = RIGHT_JOINT;
  JointChannels_[LEFT]   = LEFT_JOINT;
  JointChannels_[TWIST]  = TWIST_JOINT;
  JointChannels_[WRIST]  = WRIST_JOINT;
  JointChannels_[FINGER] = FINGER_JOINT;

  /* === Pin table setup === */
  BaseAngle_[PIN_MIN]   = BASE_MIN_ANGLE_PIN;
  BaseAngle_[PIN_MAX]   = BASE_MAX_ANGLE_PIN;

  RightAngle_[PIN_MIN]  = RIGHT_MIN_ANGLE_PIN;
  RightAngle_[PIN_MAX]  = RIGHT_MAX_ANGLE_PIN;

  LeftAngle_[PIN_MIN]   = LEFT_MIN_ANGLE_PIN;
  LeftAngle_[PIN_MAX]   = LEFT_MAX_ANGLE_PIN;

  TwistAngle_[PIN_MIN]  = TWIST_MIN_ANGLE_PIN;
  TwistAngle_[PIN_MAX]  = TWIST_MAX_ANGLE_PIN;

  WristAngle_[PIN_MIN]  = WRIST_MIN_ANGLE_PIN;
  WristAngle_[PIN_MAX]  = WRIST_MAX_ANGLE_PIN;

  FingerAngle_[PIN_MIN] = FINGER_MIN_ANGLE_PIN;
  FingerAngle_[PIN_MAX] = FINGER_MAX_ANGLE_PIN;

  PinTable_[BASE]   = BaseAngle_;
  PinTable_[RIGHT]  = RightAngle_;
  PinTable_[LEFT]   = LeftAngle_;
  PinTable_[TWIST]  = TwistAngle_;
  PinTable_[WRIST]  = WristAngle_;
  PinTable_[FINGER] = FingerAngle_;

  /* === Clear arrays === */
  for (UI_8 i = FIRST_JOINT; i < MAX_JOINT; i++) {
    CurrentEEPROMValue_[i] = EMPTY_VALUE;
    UserInputJointData_[i] = EMPTY_VALUE;

    previousJointPulse_[i]  = COMMON_MIN_ANGLE;
    currentJointPulse_[i]   = COMMON_MIN_ANGLE;
    incomingJointPulse_[i]  = COMMON_MIN_ANGLE;
    jointMotionComplete_[i] = D_FALSE;

    startTime_[i]      = 0;
    lastTarget_[i]     = 0;
    currentOutput_[i]  = COMMON_MIN_ANGLE;
  }

  /* === Queue buffer === */
  for (UI_8 j = FIRST_JOINT; j < MAX_JOINT; j++) {
    for (UI_8 k = 0; k < TRAJECTORY_QUEUE_SIZE; k++) {
      queueBuffer_[j][k] = EMPTY_VALUE;
    }
  }

  TrajectoryQueue_.front   = QUEUE_ERROR;
  TrajectoryQueue_.rear    = QUEUE_ERROR;
  TrajectoryQueue_.maxSize = TRAJECTORY_QUEUE_SIZE;

  TrajectoryQueue_.qBase[BASE]   = queueBuffer_[BASE];
  TrajectoryQueue_.qBase[RIGHT]  = queueBuffer_[RIGHT];
  TrajectoryQueue_.qBase[LEFT]   = queueBuffer_[LEFT];
  TrajectoryQueue_.qBase[TWIST]  = queueBuffer_[TWIST];
  TrajectoryQueue_.qBase[WRIST]  = queueBuffer_[WRIST];
  TrajectoryQueue_.qBase[FINGER] = queueBuffer_[FINGER];
}

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

UI_8 ArmControl::QueueIsEmpty_(Trajectory* queue)
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
    previousJointPulse[count] = FactoryInit[count];
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

void ArmControl::AutoCalibration_()
{
  UI_16 ServoAngleBuffer[MAX_JOINT][MAX_ANGLE] = {{EMPTY_VALUE}};

  for (UI_8 joint = FIRST_JOINT; joint < MAX_JOINT; joint++) {
    for (UI_8 index = FIRST_ANGLE; index < MAX_ANGLE; index++) {
      ServoAngleBuffer[joint][index] = GetTravelPoint_(joint, index);
    }
  }

  // TODO: store ServoAngleBuffer -> EEPROM or print it
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

const ArmControl::CommandEntry ArmControl::commands_[] = {
  {"init",   &ArmControl::cmdInit_},
  {"report", &ArmControl::cmdReport_},
  {"mcalib", &ArmControl::cmdManualCalib_},
  {"acalib", &ArmControl::cmdAutoCalib_},
  {"test",   &ArmControl::cmdTest_}
};

const UI_8 ArmControl::commandCount_ =
  (UI_8)(sizeof(ArmControl::commands_) / sizeof(ArmControl::commands_[0]));

void ArmControl::cmdInit_()        { UpdateJointState_(); }
void ArmControl::cmdReport_()      { reportCurrentJointPositions_(); }
void ArmControl::cmdManualCalib_() { ReadManualCalib_(UserInputJointData_); }
void ArmControl::cmdAutoCalib_()   { AutoCalibration_(); }
void ArmControl::cmdTest_()        { testAllServos_(); }

/* ================================================================= */
/* ===                  Exposed Methods                          === */
/* ================================================================= */

void ArmControl::initialize()
{
  Serial.begin(SERCHNL);

  servoDriver_.begin();
  servoDriver_.setOscillatorFrequency(OCLFRQ);
  servoDriver_.setPWMFreq(SERVO_FREQUENCY);
  delay(WAITINIT);

  // initializeEEPROM_();   // enable when you want EEPROM calibration active
  initializeTriggerPin_();

  UpdateJointState_();
}

void ArmControl::handleSerialCommands()
{
  if (Serial.available() > UNAVAIL_SER) {
    String rawInput = Serial.readStringUntil('\n');

    if (rawInput.startsWith("b")) {
      HandleDataInput_(incomingJointPulse_, &TrajectoryQueue_, rawInput);
    } else {
      for (UI_8 i = INIT_COMMAND; i < commandCount_; ++i) {
        if (rawInput == commands_[i].name) {
          (this->*commands_[i].method)();
          break;
        }
      }
    }
  }
}

void ArmControl::updateMotion()
{
  updateServoPositions_();
  if (D_TRUE == isAllJointMotionComplete_()) {
    prepareNextMotionStep_();
  }
}
