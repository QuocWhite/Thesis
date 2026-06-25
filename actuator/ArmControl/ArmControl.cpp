#include "ArmControl.h"

/* ================================================================= */
/* ===                      Constructor                          === */
/* ================================================================= */

ArmControl::ArmControl()
: servoDriver_()
{
  FactoryInit_[BASE]   = FACT_BASE_ANGLE_VAL;
  FactoryInit_[RIGHT]  = FACT_RIGHT_ANGLE_VAL;
  FactoryInit_[LEFT]   = FACT_LEFT_ANGLE_VAL;
  FactoryInit_[TWIST]  = FACT_TWIST_ANGLE_VAL;
  FactoryInit_[WRIST]  = FACT_WRIST_ANGLE_VAL;
  FactoryInit_[FINGER] = FACT_FINGER_ANGLE_VAL;

  JointChannels_[BASE]   = BASE_JOINT;
  JointChannels_[RIGHT]  = RIGHT_JOINT;
  JointChannels_[LEFT]   = LEFT_JOINT;
  JointChannels_[TWIST]  = TWIST_JOINT;
  JointChannels_[WRIST]  = WRIST_JOINT;
  JointChannels_[FINGER] = FINGER_JOINT;

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

  for (UI_8 j = FIRST_JOINT; j < MAX_JOINT; j++) {
    for (UI_8 k = 0; k < TRAJECTORY_QUEUE_SIZE; k++) {
      queueBuffer_[j][k] = EMPTY_VALUE;
    }
  }

  TrajectoryQueue_.front   = QUEUE_NULL_IDX;
  TrajectoryQueue_.rear    = QUEUE_NULL_IDX;
  TrajectoryQueue_.maxSize = TRAJECTORY_QUEUE_SIZE;

  TrajectoryQueue_.qBase[BASE]   = queueBuffer_[BASE];
  TrajectoryQueue_.qBase[RIGHT]  = queueBuffer_[RIGHT];
  TrajectoryQueue_.qBase[LEFT]   = queueBuffer_[LEFT];
  TrajectoryQueue_.qBase[TWIST]  = queueBuffer_[TWIST];
  TrajectoryQueue_.qBase[WRIST]  = queueBuffer_[WRIST];
  TrajectoryQueue_.qBase[FINGER] = queueBuffer_[FINGER];
}

/* ================================================================= */
/* ===              Conversion Helpers (Member)                   === */
/* ================================================================= */

UI_16 ArmControl::constrain_(UI_16 value)
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

UI_16 ArmControl::angleToValue_(UI_16 angle, UI_16 minTarget, UI_16 maxTarget)
{
  UI_16 convertedAngle = constrain_(angle);
  UI_16 value = (convertedAngle - SERVO_MIN_SOURCE)
              * (maxTarget - minTarget)
              / (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE) + minTarget;
  return value;
}

UI_16 ArmControl::valueToAngle_(UI_16 value, UI_16 minTarget, UI_16 maxTarget)
{
  UI_16 angle = ((value - minTarget) * (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE))
              / (maxTarget - minTarget) + SERVO_MIN_SOURCE;
  return angle;
}

/* ================================================================= */
/* ===              Queue Utilities (Member)                      === */
/* ================================================================= */

UI_8 ArmControl::queueIsFull_(Trajectory* queue)
{
  return (queue->front == (queue->rear + INCREMENT) % queue->maxSize) ? D_TRUE : D_FALSE;
}

UI_8 ArmControl::queueIsEmpty_(Trajectory* queue)
{
  return (queue->front == QUEUE_NULL_IDX) ? D_TRUE : D_FALSE;
}

void ArmControl::enqueue_(Trajectory* queue, UI_16* data)
{
  if ((D_FALSE == queueIsFull_(queue)) || (D_TRUE == queueIsEmpty_(queue))) {
    queue->front = QUEUE_EMPTY;
    queue->rear = (queue->rear + INCREMENT) % queue->maxSize;
    for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
      queue->qBase[count][queue->rear] = data[count];
    }
  }
}

void ArmControl::dequeue_(Trajectory* queue, UI_16* dataOut)
{
  if (D_FALSE == queueIsEmpty_(queue)) {
    for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
      dataOut[count] = queue->qBase[count][queue->front];
    }
    if (queue->front == queue->rear) {
      queue->front = QUEUE_NULL_IDX;
      queue->rear  = QUEUE_NULL_IDX;
    } else {
      queue->front = (queue->front + INCREMENT) % queue->maxSize;
    }
  }
}

/* ================================================================= */
/* ===              Servo and Motion State Utilities              === */
/* ================================================================= */

void ArmControl::testAllServos_()
{
  UI_16 pulseMin = COMMON_MIN_ANGLE;
  UI_16 pulseMid = (COMMON_MIN_ANGLE + COMMON_MAX_ANGLE) / MIDPOINT_DIV;

  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    if (FINGER == count) {
      servoDriver_.writeMicroseconds(JointChannels_[count], FACT_FINGER_MIN_ANGLE);
      delay(DELAY_TIME);
      servoDriver_.writeMicroseconds(JointChannels_[count], FACT_FINGER_MAX_ANGLE);
      delay(DELAY_TIME);
    } else {
      servoDriver_.writeMicroseconds(JointChannels_[count], pulseMin);
      delay(DELAY_TIME);
      servoDriver_.writeMicroseconds(JointChannels_[count], pulseMid);
      delay(DELAY_TIME);
    }
  }
}

UI_8 ArmControl::isAllJointMotionComplete_()
{
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    if (D_FALSE == jointMotionComplete_[count]) {
      return D_FALSE;
    }
  }
  return D_TRUE;
}

void ArmControl::prepareNextMotionStep_()
{
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    previousJointPulse_[count] = currentJointPulse_[count];
    jointMotionComplete_[count] = D_FALSE;
  }
  if (D_FALSE == queueIsEmpty_(&TrajectoryQueue_)) {
    dequeue_(&TrajectoryQueue_, currentJointPulse_);
  }
}

void ArmControl::updateServoPositions_()
{
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    servoDriver_.writeMicroseconds(
      JointChannels_[count],
      computeCubicTrajectory_(previousJointPulse_[count], currentJointPulse_[count], count));
  }
}

UI_16 ArmControl::computeCubicTrajectory_(UI_16 startPos, UI_16 endPos, UI_8 jointIdx)
{
  SI_32 delta = (SI_32)endPos - (SI_32)startPos;

  if (lastTarget_[jointIdx] != endPos) {
    startTime_[jointIdx] = millis();
    lastTarget_[jointIdx] = endPos;
  }

  UI_16 elapsedMs = millis() - startTime_[jointIdx];
  SI_32 T = MOTION_DURATION;
  SI_32 t = elapsedMs;

  SI_32 a0 = (SI_32)startPos * CUBIC_SCALE_POS;
  SI_32 a1 = CUBIC_ZERO;
  SI_32 a2 = (SI_32)((SI_64)CUBIC_COEFF_A2 * delta * CUBIC_SCALE_A2 / (T * T));
  SI_32 a3 = (SI_32)((SI_64)CUBIC_COEFF_A3 * delta * CUBIC_SCALE_A3 / (T * T * T));

  if (t <= T) {
    SI_32 pos = a0 / CUBIC_SCALE_POS
              + (SI_32)((SI_64)a2 * t * t / CUBIC_SCALE_A2)
              + (SI_32)((SI_64)a3 * t * t * t / CUBIC_SCALE_A3);
    if (pos < (SI_32)COMMON_MIN_ANGLE) pos = COMMON_MIN_ANGLE;
    if (pos > (SI_32)COMMON_MAX_ANGLE) pos = COMMON_MAX_ANGLE;
    currentOutput_[jointIdx] = (UI_16)pos;
  } else {
    jointMotionComplete_[jointIdx] = D_TRUE;
  }

  return currentOutput_[jointIdx];
}

void ArmControl::updateJointState_()
{
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    previousJointPulse_[count] = FactoryInit_[count];
  }
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    currentJointPulse_[count] = previousJointPulse_[count];
    servoDriver_.writeMicroseconds(JointChannels_[count], currentJointPulse_[count]);
  }
}

void ArmControl::reportCurrentJointPositions_()
{
  Serial.print("Current joint pulses: ");
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    UI_16 pulseMin = COMMON_MIN_ANGLE;
    UI_16 pulseMax = COMMON_MAX_ANGLE;
    if (FINGER == count) {
      pulseMin = FACT_FINGER_MIN_ANGLE;
      pulseMax = FACT_FINGER_MAX_ANGLE;
    }
    UI_16 val = valueToAngle_(currentJointPulse_[count], pulseMin, pulseMax);
    Serial.print(val);
    Serial.print(count < FINGER ? ", " : "\n");
  }
}

/* ================================================================= */
/* ===              Input Handling                                === */
/* ================================================================= */

void ArmControl::handleDataInput_(UI_16* value, Trajectory* Queue, String rawInput)
{
  UI_16 base    = rawInput.substring(rawInput.indexOf('b') + 1, rawInput.indexOf('r')).toInt();
  UI_16 right   = rawInput.substring(rawInput.indexOf('r') + 1, rawInput.indexOf('l')).toInt();
  UI_16 left    = rawInput.substring(rawInput.indexOf('l') + 1, rawInput.indexOf('t')).toInt();
  UI_16 twist   = rawInput.substring(rawInput.indexOf('t') + 1, rawInput.indexOf('w')).toInt();
  UI_16 wrist   = rawInput.substring(rawInput.indexOf('w') + 1, rawInput.indexOf('f')).toInt();
  UI_16 finger  = rawInput.substring(rawInput.indexOf('f') + 1).toInt();

  value[BASE]   = angleToValue_(base,   FACT_BASE_MIN_ANGLE,   FACT_BASE_MAX_ANGLE);
  value[RIGHT]  = angleToValue_(right,  FACT_RIGHT_MIN_ANGLE,  FACT_RIGHT_MAX_ANGLE);
  value[LEFT]   = angleToValue_(left,   FACT_LEFT_MIN_ANGLE,   FACT_LEFT_MAX_ANGLE);
  value[TWIST]  = angleToValue_(twist,  FACT_TWIST_MIN_ANGLE,  FACT_TWIST_MAX_ANGLE);
  value[WRIST]  = angleToValue_(wrist,  FACT_WRIST_MIN_ANGLE,  FACT_WRIST_MAX_ANGLE);
  value[FINGER] = angleToValue_(finger, FACT_FINGER_MIN_ANGLE, FACT_FINGER_MAX_ANGLE);

  enqueue_(Queue, value);
}

void ArmControl::readManualCalib_(UI_16* value)
{
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    UI_8 low  = Serial.read();
    UI_8 high = Serial.read();
    value[count] = (high << SHIFT_BYTE) | low;
  }
  writeDataToEEPROM_(value, MAGIC_MODF_VAL);
  readJointDataFromEEPROM_(CurrentEEPROMValue_);
}

void ArmControl::autoCalibration_()
{
  UI_16 ServoAngleBuffer[MAX_JOINT][MAX_ANGLE] = {{EMPTY_VALUE}};

  for (UI_8 joint = FIRST_JOINT; joint < MAX_JOINT; joint++) {
    for (UI_8 index = FIRST_ANGLE; index < MAX_ANGLE; index++) {
      ServoAngleBuffer[joint][index] = getTravelPoint_(joint, index);
    }
  }
}

/* ================================================================= */
/* ===              EEPROM Utilities                              === */
/* ================================================================= */

void ArmControl::writeDataToEEPROM_(const UI_16* value, UI_8 magic_val)
{
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    EEPROM.write(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + MSB_OFFSET,
                 (value[count] >> SHIFT_BYTE) & BYTE_MASK);
    EEPROM.write(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + LSB_OFFSET,
                 value[count] & BYTE_MASK);
  }
  EEPROM.write(MAGIC_ADDR, magic_val);
  EEPROM.commit();
}

void ArmControl::readJointDataFromEEPROM_(UI_16* value)
{
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    UI_8 msb = EEPROM.read(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + MSB_OFFSET);
    UI_8 lsb = EEPROM.read(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + LSB_OFFSET);
    value[count] = ((msb << SHIFT_BYTE) | lsb);
  }
}

void ArmControl::initializeEEPROM_()
{
  if (!EEPROM.begin(EEPROM_SIZE)) {
    return;
  }

  UI_8 eepVal = EEPROM.read(MAGIC_ADDR);
  if ((eepVal != MAGIC_INIT_VAL) && (eepVal != MAGIC_MODF_VAL)) {
    writeDataToEEPROM_(FactoryInit_, MAGIC_INIT_VAL);
  }
  readJointDataFromEEPROM_(CurrentEEPROMValue_);
}

/* ================================================================= */
/* ===              System Input Data (Triggers)                  === */
/* ================================================================= */

void ArmControl::initializeTriggerPin_()
{
  for (UI_8 joint = FIRST_JOINT; joint < MAX_JOINT; joint++) {
    for (UI_8 index = FIRST_ANGLE; index < MAX_ANGLE; index++) {
      pinMode(PinTable_[joint][index], INPUT);
    }
  }
}

UI_8 ArmControl::getTriggerData_(UI_8 joint, UI_8 index)
{
  return digitalRead(PinTable_[joint][index]);
}

UI_16 ArmControl::getTravelPoint_(UI_8 joint, UI_8 pin)
{
  UI_8 triggerData = getTriggerData_(joint, pin);
  UI_16 start = (SERVO_MIN_TARGET + SERVO_MAX_TARGET) / MIDPOINT_DIV;
  UI_16 angle = start;
  UI_16 end;
  UI_8  step;

  pinMode(PinTable_[joint][pin], INPUT);

  if (PIN_MIN == pin) {
    end  = SERVO_MIN_TARGET;
    step = -STEP_SIZE;
  } else if (PIN_MAX == pin) {
    end  = SERVO_MAX_TARGET;
    step = STEP_SIZE;
  } else {
    end = start;
  }

  UI_16 ret = start;
  for (; (pin ? angle <= end : angle >= end); angle += step) {
    servoDriver_.writeMicroseconds(JointChannels_[joint], angle);
    delay(DELAY_SET);
    if (D_ON == triggerData) {
      ret = angle;
    }
  }

  return ret;
}

/* ================================================================= */
/* ===              Command Table                                 === */
/* ================================================================= */

const ArmControl::CommandEntry ArmControl::commands_[] = {
  {"init",   &ArmControl::cmdInit_},
  {"report", &ArmControl::cmdReport_},
  {"mcalib", &ArmControl::cmdManualCalib_},
  {"acalib", &ArmControl::cmdAutoCalib_},
  {"test",   &ArmControl::cmdTest_}
};

const UI_8 ArmControl::commandCount_ =
  (UI_8)(sizeof(ArmControl::commands_) / sizeof(ArmControl::commands_[0]));

void ArmControl::cmdInit_()        { updateJointState_(); }
void ArmControl::cmdReport_()      { reportCurrentJointPositions_(); }
void ArmControl::cmdManualCalib_() { readManualCalib_(UserInputJointData_); }
void ArmControl::cmdAutoCalib_()   { autoCalibration_(); }
void ArmControl::cmdTest_()        { testAllServos_(); }

/* ================================================================= */
/* ===                  Exposed Methods                           === */
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

  updateJointState_();
}

void ArmControl::handleSerialCommands()
{
  if (Serial.available() > SERIAL_AVAIL_MIN) {
    String rawInput = Serial.readStringUntil('\n');

    if (rawInput.startsWith("b")) {
      handleDataInput_(incomingJointPulse_, &TrajectoryQueue_, rawInput);
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
