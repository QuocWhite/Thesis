#include "ArmControl.h"

using namespace ArmControl;

enum {
  BASE = 0,
  RIGHT,
  LEFT,
  TWIST,
  WRIST,
  FINGER,
  PWM,
  MAX_CHANNEL 
};

static Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

static UI_8 jointChannels[MAX_CHANNEL] = {BASE_JOINT, RIGHT_JOINT, LEFT_JOINT, TWIST_JOINT, WRIST_JOINT, FINGER_JOINT};

static SI_16 factory_angle_matrix [MAX_JOINT][MAX_ANGLE] = {
  {-109, 109},
  {0, 218},
  {0, 218},
  {0, 218},
  {0, 198},
  {0, 206}
};

static SI_16 angle_matrix [MAX_JOINT][MAX_ANGLE];

// === Queue Structure ===
typedef struct {
  SI_8 front, rear, maxSize;
  UI_16 *qBase[7];
} TrajectoryQueue;

static UI_16 queueBuffer[7][TRAJECTORY_QUEUE_SIZE] = {{0}};

static TrajectoryQueue trajectoryQueue = {
  QUEUE_EMPTY, QUEUE_EMPTY, TRAJECTORY_QUEUE_SIZE,
  {queueBuffer[BASE], queueBuffer[RIGHT], queueBuffer[LEFT],
   queueBuffer[TWIST], queueBuffer[WRIST], queueBuffer[FINGER], queueBuffer[MAX_CHANNEL]}
};

// === Motion State ===
static UI_16 previousJointPulse[MAX_JOINT] = {SERVO_PWM_MIN};
static UI_16 currentJointPulse[MAX_JOINT]  = {SERVO_PWM_MIN};
static UI_16 incomingJointPulse[MAX_JOINT] = {SERVO_PWM_MIN};
static float motionDuration_ms = MOTION_DURATION;
static bool jointMotionComplete[MAX_JOINT] = {false};

// === Queue Utils === */
static bool Queue_IsFull(TrajectoryQueue *queue) {
  return (queue->front == (queue->rear + INCREASEMENT) % queue->maxSize);
}

static bool Queue_IsEmpty(TrajectoryQueue *queue) {
  return queue->front == QUEUE_EMPTY;
}

static void Queue_Enqueue(TrajectoryQueue *queue, UI_16 *data) {
  if (!Queue_IsFull(queue)) {
    if (Queue_IsEmpty(queue)) {
      queue->front = 0;
      queue->rear = (queue->rear + INCREASEMENT) % queue->maxSize;
      for (int i = COUNTER_INIT; i < MAX_CHANNEL; i++) {
        queue->qBase[i][queue->rear] = data[i];
      }
    }
  }
}

static void Queue_Dequeue(TrajectoryQueue *queue, UI_16 *dataOut) {
  if (!Queue_IsEmpty(queue)) {
    for (int i = COUNTER_INIT; i < END_COUNTER; i++) {
      dataOut[i] = queue->qBase[i][queue->front];
    }
    if (queue->front == queue->rear) {
      queue->front = QUEUE_EMPTY;
      queue->rear = QUEUE_EMPTY;
    } else {
      queue->front = (queue->front + INCREASEMENT) % queue->maxSize;
    }
  }
}

// === Internal Utilities ===
static void testAllServos() {
  for (int i = COUNTER_INIT; i < MAX_CHANNEL; i++) {
    int pulseMin = SERVO_PWM_MIN;
    int pulseMid = (SERVO_PWM_MIN + SERVO_PWM_MAX) / 2;
    servoDriver.writeMicroseconds(jointChannels[i], pulseMin);
    delay(DELAY_TIME_MS); 
    servoDriver.writeMicroseconds(jointChannels[i], pulseMid); 
    delay(DELAY_TIME_MS);
  }
}

static bool isAllJointMotionComplete() {
  for (int i = COUNTER_INIT; i < MAX_CHANNEL; i++) {
    if (!jointMotionComplete[i]) return false;
  }
  return true;
}

static void prepareNextMotionStep() {
  for (int i = COUNTER_INIT; i < MAX_CHANNEL; i++) {
    previousJointPulse[i] = currentJointPulse[i];
    jointMotionComplete[i] = false;
  }
  if (!Queue_IsEmpty(&trajectoryQueue)) {
    Queue_Dequeue(&trajectoryQueue, currentJointPulse);
  }
}

static UI_16 computeCubicTrajectory(UI_16 start, UI_16 end, int jointIdx) {
  static unsigned long startTime[MAX_CHANNEL];
  static UI_16 previousEnd[6];
  static UI_16 outputValue[6];

  float t = (millis() - startTime[jointIdx]) / 1000.0;
  float duration = motionDuration_ms / 1000.0;

  if (previousEnd[jointIdx] != end) {
    startTime[jointIdx] = millis();
    previousEnd[jointIdx] = end;
  }

  float a0 = start;
  float a1 = 0;
  float a2 = 3.0 / (duration * duration) * (end - start);
  float a3 = -2.0 / (duration * duration * duration) * (end - start);

  if (t <= duration) {
    outputValue[jointIdx] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  } else {
    jointMotionComplete[jointIdx] = true;
  }

  return outputValue[jointIdx];
}

static void updateServoPositions() {
  for (int i = 0; i < MAX_JOINT; i++) {
    servoDriver.writeMicroseconds(jointChannels[i], computeCubicTrajectory(previousJointPulse[i], currentJointPulse[i], i));
  }
}

static void initializeJointState() {
  previousJointPulse[BASE]   = map(0, -109, 109, SERVO_PWM_MIN, SERVO_PWM_MAX);
  previousJointPulse[RIGHT]  = map(37, 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  previousJointPulse[LEFT]   = map(127, 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  previousJointPulse[TWIST]  = map(109, 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  previousJointPulse[WRIST]  = map(11, 0, 198, SERVO_PWM_MAX, SERVO_PWM_MIN);
  previousJointPulse[FINGER] = map(108, 0, 206, SERVO_PWM_MIN, SERVO_PWM_MAX);

  for (int i = 0; i < 6; i++) {
    currentJointPulse[i] = previousJointPulse[i];
    servoDriver.writeMicroseconds(jointChannels[i], previousJointPulse[i]);
  }
  currentJointPulse[6] = 150;
}

static void wite_defaultEEPROM( SI_16 list[MAX_JOINT][MAX_ANGLE] ) {
  UI_8 row_index;
  UI_8 column_index;

  for ( row_index = 0U; row_index < MAX_JOINT; row_index++ ) {
    for ( column_index = 0U; column_index < MAX_ANGLE; column_index++ ) {
      list[row_index][column_index] = factory_angle_matrix[row_index][column_index];
    }
  }
}

/* === Initialize EEPROM === */
static void initializeEEPROM() {
  /* Error case */
  if (!EEPROM.begin(EEPROM_SIZE)) {
    return;
  }
  if (EEPROM.read(MAGIC_ADDR) != MAGIC_VAL) {
    wite_defaultEEPROM(angle_matrix);
    EEPROM.write(MAGIC_ADDR, MAGIC_VAL);
    EEPROM.commit();
  }
}

// === Exposed Methods ===

void ArmControl::initialize() {
  Serial.begin(115200);
  servoDriver.begin();
  servoDriver.setOscillatorFrequency(27000000);
  servoDriver.setPWMFreq(SERVO_FREQUENCY);
  delay(10);

  initializeJointState();

  initializeEEPROM();

}

void ArmControl::handleSerialCommands() {
  if (Serial.available() > 0) {
    String rawInput = Serial.readStringUntil('\n');
    if (rawInput.startsWith("s")) {
      int j1 = rawInput.substring(1, rawInput.indexOf('a')).toInt();
      int j2 = rawInput.substring(rawInput.indexOf('a') + 1, rawInput.indexOf('b')).toInt();
      int j3 = rawInput.substring(rawInput.indexOf('b') + 1, rawInput.indexOf('c')).toInt();
      int j4 = rawInput.substring(rawInput.indexOf('c') + 1, rawInput.indexOf('d')).toInt();
      int j5 = rawInput.substring(rawInput.indexOf('d') + 1, rawInput.indexOf('e')).toInt();
      int j6 = rawInput.substring(rawInput.indexOf('e') + 1, rawInput.indexOf('f')).toInt();
      int gripper = rawInput.substring(rawInput.indexOf('f') + 1).toInt();

      incomingJointPulse[BASE]   = map(constrain(j1 + 109, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[RIGHT]  = map(constrain(172 - j2, 5, 140), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[LEFT]   = map(constrain(j3 + j2 + 37, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[TWIST]  = map(constrain(j4 + 109, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[WRIST]  = map(constrain(j5 + 101, 0, 198), 0, 198, SERVO_PWM_MAX, SERVO_PWM_MIN);
      incomingJointPulse[FINGER] = map(constrain(j6 + 108, 0, 206), 0, 206, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[6] = gripper;

      Queue_Enqueue(&trajectoryQueue, incomingJointPulse);
    } else if (rawInput == "init") {
      initializeJointState();
    } else if (rawInput == "report") {
      reportCurrentJointPositions();
    }
  }
}

void ArmControl::updateMotion() {
  updateServoPositions();
  if (isAllJointMotionComplete()) {
    prepareNextMotionStep();
  }
}

void ArmControl::reportCurrentJointPositions() {
  Serial.print("Current joint pulses: ");
  for (int i = COUNTER_INIT; i < MAX_CHANNEL; i++) {
    Serial.print(currentJointPulse[i]);
    Serial.print(i < FINGER ? ", " : "\n");
  }
  Serial.print("Gripper: ");
  Serial.println(currentJointPulse[MAX_CHANNEL]);
}

