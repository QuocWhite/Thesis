#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// === Servo PWM Range Constants ===
#define SERVO_PWM_MIN      360
#define SERVO_PWM_MAX     2550
#define SERVO_FREQUENCY     50

// === Queue Settings ===
#define TRAJECTORY_QUEUE_SIZE  16

// === Servo Channel Assignments ===
#define BASE_JOINT     12
#define RIGHT_JOINT     0
#define LEFT_JOINT      1
#define TWIST_JOINT     8
#define WRIST_JOINT     7
#define FINGER_JOINT    5

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

// Mapping joint index to PWM channel
uint8_t jointChannels[6] = {
  BASE_JOINT, RIGHT_JOINT, LEFT_JOINT,
  TWIST_JOINT, WRIST_JOINT, FINGER_JOINT
};

// === Circular Queue Definition ===
typedef struct {
  int8_t front, rear, maxSize;
  uint16_t *qBase[7]; // One for each joint + gripper
} TrajectoryQueue;

// Queue Buffers
uint16_t queueBuffer[7][TRAJECTORY_QUEUE_SIZE] = {{0}};

// Instantiate the queue
TrajectoryQueue trajectoryQueue = {
  -1, -1, TRAJECTORY_QUEUE_SIZE,
  {
    queueBuffer[0], queueBuffer[1], queueBuffer[2],
    queueBuffer[3], queueBuffer[4], queueBuffer[5], queueBuffer[6]
  }
};

// === Motion State Variables ===
uint16_t previousJointPulse[6] = {SERVO_PWM_MIN};
uint16_t currentJointPulse[7]  = {SERVO_PWM_MIN, SERVO_PWM_MIN, SERVO_PWM_MIN,
                                  SERVO_PWM_MIN, SERVO_PWM_MIN, SERVO_PWM_MIN, 150};
uint16_t incomingJointPulse[7] = {SERVO_PWM_MIN};
float motionDuration_ms = 150;
bool jointMotionComplete[6] = {false};

// === Queue Utility Functions ===
bool Queue_IsFull(TrajectoryQueue *queue) {
  return (queue->front == (queue->rear + 1) % queue->maxSize);
}

bool Queue_IsEmpty(TrajectoryQueue *queue) {
  return queue->front == -1;
}

void Queue_Enqueue(TrajectoryQueue *queue, uint16_t *data) {
  if (!Queue_IsFull(queue)) {
    if (queue->front == -1) queue->front = 0;
    queue->rear = (queue->rear + 1) % queue->maxSize;
    for (int i = 0; i < 7; i++) {
      queue->qBase[i][queue->rear] = data[i];
    }
  }
}

void Queue_Dequeue(TrajectoryQueue *queue, uint16_t *dataOut) {
  if (!Queue_IsEmpty(queue)) {
    for (int i = 0; i < 7; i++) {
      dataOut[i] = queue->qBase[i][queue->front];
    }
    if (queue->front == queue->rear) {
      queue->front = -1;
      queue->rear = -1;
    } else {
      queue->front = (queue->front + 1) % queue->maxSize;
    }
  }
}

// === Servo Testing ===
void testSingleServo(uint8_t channel) {
  int pulseMin, pulseMid;
    pulseMin = SERVO_PWM_MIN;
    pulseMid = (SERVO_PWM_MIN + SERVO_PWM_MAX) / 2;

  Serial.print("Testing Servo Channel: ");
  Serial.println(channel);
  servoDriver.writeMicroseconds(channel, pulseMin); delay(500);
  servoDriver.writeMicroseconds(channel, pulseMid); delay(500);
}

void testAllServos() {
  for (int i = 0; i < 6; i++) {
    testSingleServo(jointChannels[i]);
  }
}

// === Serial Input Parsing ===
void readSerialAndQueueTrajectory() {
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

      incomingJointPulse[0] = map(constrain(j1 + 109, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[1] = map(constrain(172 - j2, 5, 140), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[2] = map(constrain(j3 + j2 + 37, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[3] = map(constrain(j4 + 109, 0, 218), 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[4] = map(constrain(j5 + 101, 0, 198), 0, 198, SERVO_PWM_MAX, SERVO_PWM_MIN);
      incomingJointPulse[5] = map(constrain(j6 + 108, 0, 206), 0, 206, SERVO_PWM_MIN, SERVO_PWM_MAX);
      incomingJointPulse[6] = gripper;

      Queue_Enqueue(&trajectoryQueue, incomingJointPulse);
    }
    else if (rawInput == "init") {
      initializeJointState();
    }
    else {
      /* Do nothing*/
    }
  }
}

// === Trajectory Motion Logic ===
bool isAllJointMotionComplete() {
  for (int i = 0; i < 6; i++) {
    if (!jointMotionComplete[i]) return false;
  }
  return true;
}

void prepareNextMotionStep() {
  for (int i = 0; i < 6; i++) {
    previousJointPulse[i] = currentJointPulse[i];
    jointMotionComplete[i] = false;
  }
  if (!Queue_IsEmpty(&trajectoryQueue)) {
    Queue_Dequeue(&trajectoryQueue, currentJointPulse);
  }
}

uint16_t computeCubicTrajectory(uint16_t start, uint16_t end, int jointIdx) {
  static unsigned long startTime[6];
  static uint16_t previousEnd[6];
  static uint16_t outputValue[6];

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

void updateServoPositions() {
  for (int i = 0; i < 6; i++) {
    servoDriver.writeMicroseconds(jointChannels[i], computeCubicTrajectory(previousJointPulse[i], currentJointPulse[i], i));
  }
}

// === Initialization ===
void initializeJointState() {
  previousJointPulse[0] = map(0, -109, 109, SERVO_PWM_MIN, SERVO_PWM_MAX);
  previousJointPulse[1] = map(37, 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  previousJointPulse[2] = map(127, 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  previousJointPulse[3] = map(109, 0, 218, SERVO_PWM_MIN, SERVO_PWM_MAX);
  previousJointPulse[4] = map(11, 0, 198, SERVO_PWM_MAX, SERVO_PWM_MIN);
  previousJointPulse[5] = map(108, 0, 206, SERVO_PWM_MIN, SERVO_PWM_MAX);

  for (int i = 0; i < 6; i++) {
    currentJointPulse[i] = previousJointPulse[i];
    servoDriver.writeMicroseconds(jointChannels[i], previousJointPulse[i]);
  }
  currentJointPulse[6] = 150;
}

void setup() {
  Serial.begin(115200);
  servoDriver.begin();
  servoDriver.setOscillatorFrequency(27000000);
  servoDriver.setPWMFreq(SERVO_FREQUENCY);
  delay(10);

  Serial.println("Starting servo self-test...");
  testAllServos();

  initializeJointState();
}

void loop() {
  readSerialAndQueueTrajectory();
  updateServoPositions();
  if (isAllJointMotionComplete()) {
    prepareNextMotionStep();
  }
}

