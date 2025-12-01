#include "ArmControl.h"

/* === Constructor and Command Map === */
ArmControl::ArmControl() {
  commands = {
    {"init",   [this](){ this->UpdateJointState(); }},
    {"report", [this](){ this->reportCurrentJointPositions(); }},
    {"mcalib", [this](){ this->ReadManualCalib(this->UserInputJointData); }},
    {"acalib", [this](){ this->AutoCalibration(); }},
    {"test",   [this](){ this->testAllServos(); }}
  };
}
/// --- End of ArmControl::ArmControl() ---

/* === Common Functions === */
UI_16 ArmControl::AngletoValue(UI_16 angle, UI_16 minTarget, UI_16 maxTarget) {
  UI_16 convertedAngle = ConsTrain(angle);
  return (convertedAngle - SERVO_MIN_SOURCE) * (maxTarget - minTarget) / (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE) + minTarget;
}
/// --- End of AngletoValue() ---

UI_16 ArmControl::ConsTrain(UI_16 value) {
  if (value < SERVO_MIN_SOURCE) return SERVO_MIN_SOURCE;
  if (value > SERVO_MAX_SOURCE) return SERVO_MAX_SOURCE;
  return value;
}
/// --- End of ConsTrain() ---

UI_16 ArmControl::ValuetoAngle(UI_16 value, UI_16 minTarget, UI_16 maxTarget) {
  return ((value - minTarget) * (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE)) / (maxTarget - minTarget) + SERVO_MIN_SOURCE;
}
/// --- End of ValuetoAngle() ---

/* === Queue Utilities === */
UI_8 ArmControl::QueueIsFull(Trajectory *queue) {
  return (queue->front == (queue->rear + INCREASE) % queue->maxSize) ? D_TRUE : D_FALSE;
}
/// --- End of QueueIsFull() ---

UI_8 ArmControl::QueueIsEmpty(Trajectory *queue) {
  return (queue->front == QUEUE_ERROR) ? D_TRUE : D_FALSE;
}
/// --- End of QueueIsEmpty() ---

void ArmControl::QueueEndQueue(Trajectory *queue, UI_16 *data) {
  if ((D_FALSE == QueueIsFull(queue)) || (D_TRUE == QueueIsEmpty(queue))) {
    queue->front = QUEUE_EMPTY;
    queue->rear = (queue->rear + INCREASE) % queue->maxSize;
    for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++)
      queue->qBase[count][queue->rear] = data[count];
    }
}
/// --- End of QueueEndQueue() ---

void ArmControl::QueueDequeue(Trajectory *queue, UI_16 *dataOut) {
  if (D_FALSE == QueueIsEmpty(queue)) {
    for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++)
      dataOut[count] = queue->qBase[count][queue->front];
      if (queue->front == queue->rear) {
        queue->front = QUEUE_ERROR;
        queue->rear = QUEUE_ERROR;
      } else {
        queue->front = (queue->front + INCREASE) % queue->maxSize;
      }
  }
}
/// --- End of QueueDequeue() ---

/* === Servo and Motion State Utilities === */
void ArmControl::testAllServos() {
  UI_16 pulseMin = COMMON_MIN_ANGLE;
  UI_16 pulseMid = (COMMON_MIN_ANGLE + COMMON_MAX_ANGLE) / MED_DEVIDE;

  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    if (FINGER == count) {
      pulseMin = FACT_FINGER_MIN_ANGLE;
      pulseMid = FACT_FINGER_MAX_ANGLE;
    }
    servoDriver.writeMicroseconds(JointChannels[count], pulseMin);
    delay(DELAY_TIME);
    servoDriver.writeMicroseconds(JointChannels[count], pulseMid);
    delay(DELAY_TIME);
  }
}
/// --- End of testAllServos() ---

UI_8 ArmControl::isAllJointMotionComplete() {
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++)
    if (D_FALSE == jointMotionComplete[count]) return D_FALSE;
  return D_TRUE;
}
/// --- End of isAllJointMotionComplete() ---

void ArmControl::prepareNextMotionStep() {
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    previousJointPulse[count] = currentJointPulse[count];
    jointMotionComplete[count] = D_FALSE;
  }
  if (D_FALSE == QueueIsEmpty(&TrajectoryQueue))
    QueueDequeue(&TrajectoryQueue, currentJointPulse);
}
/// --- End of prepareNextMotionStep() ---

void ArmControl::updateServoPositions() {
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++)
    servoDriver.writeMicroseconds(JointChannels[count],
    computeCubicTrajectory(previousJointPulse[count],
    currentJointPulse[count], count));
}
/// --- End of updateServoPositions() ---

UI_16 ArmControl::computeCubicTrajectory(UI_16 startPos, UI_16 endPos, UI_8 jointIdx) {
  static UI_16 startTime[MAX_JOINT];
  static UI_16 lastTarget[MAX_JOINT];
  static UI_16 currentOutput[MAX_JOINT];
  SI_32 delta = (SI_32)endPos - (SI_32)startPos;
  if (lastTarget[jointIdx] != endPos) {
    startTime[jointIdx] = millis();
    lastTarget[jointIdx] = endPos;
  }
  UI_16 elapsedMs = millis() - startTime[jointIdx];
  SI_32 T = MOTION_DURATION;
  SI_32 t = elapsedMs;
  SI_32 a0 = startPos * CUBIC_SCALE_POS;
  SI_32 a1 = CUBIC_ZERO;
  SI_32 a2 = (CUBIC_COEFF_A2 * delta * CUBIC_SCALE_A2) / (T * T);
  SI_32 a3 = (CUBIC_COEFF_A3 * delta * CUBIC_SCALE_A3) / (T * T * T);
  SI_32 pos;
  if (t <= T) {
    pos = a0 / CUBIC_SCALE_POS + (a1 * t)
        + (a2 * t * t) / CUBIC_SCALE_A2
        + (a3 * t * t * t) / CUBIC_SCALE_A3;
    currentOutput[jointIdx] = (UI_16)pos;
  } else {
    jointMotionComplete[jointIdx] = D_TRUE;
    currentOutput[jointIdx] = endPos;
  }
  return currentOutput[jointIdx];
}/// --- End of computeCubicTrajectory() ---

void ArmControl::UpdateJointState() {
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++)
    previousJointPulse[count] = CurrentEEPROMValue[count];
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    currentJointPulse[count] = previousJointPulse[count];
    servoDriver.writeMicroseconds(JointChannels[count], currentJointPulse[count]);
  }
}
/// --- End of UpdateJointState() ---

void ArmControl::reportCurrentJointPositions() {
  UI_16 pulseMin = COMMON_MIN_ANGLE;
  UI_16 pulseMax = COMMON_MAX_ANGLE;
  Serial.print("Current joint pulses: ");
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    if (FINGER == count) {
      pulseMin = FACT_FINGER_MIN_ANGLE;
      pulseMax = FACT_FINGER_MAX_ANGLE;
    }
    UI_16 val = ValuetoAngle(currentJointPulse[count], pulseMin, pulseMax);
    Serial.print(val);
    Serial.print(count < FINGER ? ", " : "\n");
  }
}
/// --- End of reportCurrentJointPositions() ---

/* === Input Handling === */
void ArmControl::HandleDataInput(UI_16 *value, Trajectory *Queue, String rawInput) {
  UI_16 base, right, left, twist, wrist, finger;
  base   = rawInput.substring(FIRST_INPUT, rawInput.indexOf('r')).toInt();
  right  = rawInput.substring(rawInput.indexOf('r') + INCREASE, rawInput.indexOf('l')).toInt();
  left   = rawInput.substring(rawInput.indexOf('l') + INCREASE, rawInput.indexOf('t')).toInt();
  twist  = rawInput.substring(rawInput.indexOf('t') + INCREASE, rawInput.indexOf('w')).toInt();
  wrist  = rawInput.substring(rawInput.indexOf('w') + INCREASE, rawInput.indexOf('f')).toInt();
  finger = rawInput.substring(rawInput.indexOf('f') + INCREASE).toInt();
  value[BASE]   = AngletoValue(base,   FACT_BASE_MIN_ANGLE,   FACT_BASE_MAX_ANGLE);
  value[RIGHT]  = AngletoValue(right,  FACT_RIGHT_MIN_ANGLE,  FACT_RIGHT_MAX_ANGLE);
  value[LEFT]   = AngletoValue(left,   FACT_LEFT_MIN_ANGLE,   FACT_LEFT_MAX_ANGLE);
  value[TWIST]  = AngletoValue(twist,  FACT_TWIST_MIN_ANGLE,  FACT_TWIST_MAX_ANGLE);
  value[WRIST]  = AngletoValue(wrist,  FACT_WRIST_MIN_ANGLE,  FACT_WRIST_MAX_ANGLE);
  value[FINGER] = AngletoValue(finger, FACT_FINGER_MIN_ANGLE, FACT_FINGER_MAX_ANGLE);
  QueueEndQueue(Queue, value);
}
/// --- End of HandleDataInput() ---

void ArmControl::ReadManualCalib(UI_16 *value) {
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    UI_8 low = Serial.read();
    UI_8 high = Serial.read();
    value[count] = (high << SHIFT_BYTE) | low;
  }
  WriteDataToEEPROM(value, MAGIC_MODF_VAL);
  ReadJointDataFromEEPROM(CurrentEEPROMValue);
}
/// --- End of ReadManualCalib() ---

/* === EEPROM Utilities === */
void ArmControl::WriteDataToEEPROM(const UI_16 *value, UI_8 magic_val) {
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    EEPROM.write(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + MSB_OFFSET, (value[count] >> SHIFT_BYTE) & BYTE_MASK);
    EEPROM.write(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + LSB_OFFSET, value[count] & BYTE_MASK);
  }
  EEPROM.write(MAGIC_ADDR, magic_val);
  EEPROM.commit();
}
/// --- End of WriteDataToEEPROM() ---

void ArmControl::ReadJointDataFromEEPROM(UI_16 *value) {
  for (UI_8 count = FIRST_JOINT; count < MAX_JOINT; count++) {
    UI_8 msb = EEPROM.read(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + MSB_OFFSET);
    UI_8 lsb = EEPROM.read(START_JOINT_ADDR + count * JOINT_BYTE_SIZE + LSB_OFFSET);
    value[count] = ((msb << SHIFT_BYTE) | lsb);
  }
}
/// --- End of ReadJointDataFromEEPROM() ---

void ArmControl::initializeEEPROM() {
  if (!EEPROM.begin(EEPROM_SIZE)) return;
  UI_8 eepVal = EEPROM.read(MAGIC_ADDR);
  if ((eepVal != MAGIC_INIT_VAL) && (eepVal != MAGIC_MODF_VAL))
    WriteDataToEEPROM(FactoryInit, MAGIC_INIT_VAL);
  ReadJointDataFromEEPROM(CurrentEEPROMValue);
}
/// --- End of initializeEEPROM() ---

/* === System input data === */
void ArmControl::initializeTriggerPin() {
  for (UI_8 joint = FIRST_JOINT; joint < MAX_JOINT; joint++)
    for (UI_8 index = FIRST_ANGLE; index < MAX_ANGLE; index++)
      pinMode(PinTable[joint][index], INPUT);
}
/// --- End of initializeTriggerPin() ---

UI_8 ArmControl::GetTriggerData(UI_8 joint, UI_8 index) {
  return digitalRead(PinTable[joint][index]);
}
/// --- End of GetTriggerData() ---

void ArmControl::AutoCalibration() {
  UI_16 ServoAngleBuffer[MAX_JOINT][MAX_ANGLE];
  for (UI_8 joint = FIRST_JOINT; joint < MAX_JOINT; joint++)
    for (UI_8 index = FIRST_ANGLE; index < MAX_ANGLE; index++)
      ServoAngleBuffer[joint][index] = GetTravelPoint(joint, index);
}
/// --- End of AutoCalibration() ---

UI_16 ArmControl::GetTravelPoint(UI_8 joint, UI_8 pin) {
  UI_8 triggerData = GetTriggerData(joint, pin);
  UI_16 start = (SERVO_MIN_TARGET + SERVO_MAX_TARGET) / MED_DEVIDE;
  UI_8 angle = start;
  UI_16 end, ret;
  UI_8 step;
  if (PIN_MIN == pin) {
    end = SERVO_MIN_TARGET;
    step = -STEP_SIZE;
  } else if (PIN_MAX == pin) {
    end = SERVO_MAX_TARGET;
    step = STEP_SIZE;
  } else {
    end = start;
  }
  for (; (pin ? angle <= end : angle >= end); angle += step) {
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
/// --- End of GetTravelPoint() ---

/* === Exposed Methods === */
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
/// --- End of initialize() ---

void ArmControl::handleSerialCommands() {
  if (Serial.available() <= UNAVAIL_SER) return;
  String rawInput = Serial.readStringUntil('\n');
  if (rawInput.startsWith("b")) {
    HandleDataInput(incomingJointPulse, &TrajectoryQueue, rawInput);
    return;
  }
  for (UI_8 count = INIT_COMMAND; count < commands.size(); ++count) {
    if (rawInput == commands[count].name) {
      commands[count].func();
      return;
    }
  }
}
/// --- End of handleSerialCommands() ---

void ArmControl::updateMotion() {
  updateServoPositions();
  if (D_TRUE == isAllJointMotionComplete())
    prepareNextMotionStep();
}
/// --- End of updateMotion() ---

