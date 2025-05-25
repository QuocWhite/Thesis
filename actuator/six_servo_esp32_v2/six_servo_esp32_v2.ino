#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMAX    520
#define USMIN       360
#define USMID       1400
#define USMAX       2550
#define SERVO_FREQ  50
#define QUEUE_SIZE  16

#define J1 12
#define J2 0
#define J3 1
#define J4 8
#define J5 7
#define J6 5

// Queue structure and storage
struct CircularQueue_Types {
  int8_t Front = -1, Rear = -1;
  uint16_t *QueArr[7];
};

uint8_t joints[6] = {J1, J2, J3, J4, J5, J6};

uint16_t Queue_Arrays[7][QUEUE_SIZE] = {0};

CircularQueue_Types TQ = {
  -1, -1,
  { Queue_Arrays[0], Queue_Arrays[1], Queue_Arrays[2],
    Queue_Arrays[3], Queue_Arrays[4], Queue_Arrays[5], Queue_Arrays[6] }
};

uint16_t j_pre[6] = {USMIN}, j_now[7] = {USMIN, USMIN, USMIN, USMIN, USMIN, USMIN, 150}, data_buf[7];
bool complete[6] = {true, true, true, true, true, true};
float tf = 150;

bool isQueueFull() {
  return (TQ.Front == (TQ.Rear + 1) % QUEUE_SIZE);
}

bool isQueueEmpty() {
  return (TQ.Front == -1);
}

void enqueue(uint16_t *data) {
  if (!isQueueFull()) {
    if (TQ.Front == -1) TQ.Front = 0;
    TQ.Rear = (TQ.Rear + 1) % QUEUE_SIZE;
    for (int i = 0; i < 7; ++i) TQ.QueArr[i][TQ.Rear] = data[i];
  }
}

void dequeue(uint16_t *out) {
  if (!isQueueEmpty()) {
    for (int i = 0; i < 7; ++i) out[i] = TQ.QueArr[i][TQ.Front];
    if (TQ.Front == TQ.Rear) TQ.Front = TQ.Rear = -1;
    else TQ.Front = (TQ.Front + 1) % QUEUE_SIZE;
  }
}

void readSerial2Data() {
  if (Serial2.available()) {
    String input = Serial2.readStringUntil('\n');
    if (input.startsWith("s")) {
      int idx[7];
      char keys[] = "abcdef";
      for (int i = 0; i < 6; ++i) {
        idx[i] = input.indexOf(keys[i]);
      }
      idx[6] = input.length();

      int vals[7];
      vals[0] = input.substring(1, idx[0]).toInt();
      for (int i = 1; i < 7; ++i)
        vals[i] = input.substring(idx[i-1]+1, idx[i]).toInt();

      data_buf[0] = map(constrain(vals[0] + 109, 0, 218), 0, 218, USMIN, USMAX);
      data_buf[1] = map(constrain(172 - vals[1], 5, 140), 0, 218, USMIN, USMAX);
      data_buf[2] = map(constrain(vals[2] + vals[1] + 37, 0, 218), 0, 218, USMIN, USMAX);
      data_buf[3] = map(constrain(vals[3] + 109, 0, 218), 0, 218, USMIN, USMAX);
      data_buf[4] = map(constrain(vals[4] + 101, 0, 198), 0, 198, USMAX, USMIN);
      data_buf[5] = map(constrain(vals[5] + 108, 0, 206), 0, 206, USMIN, USMAX);
      data_buf[6] = vals[6];

      enqueue(data_buf);
    }
  }
}

uint16_t trajectory(uint16_t start, uint16_t end, int joint) {
  float a0 = start, a1 = 0;
  float T = tf / 1000.0;
  float a2 = 3.0 / (T * T) * (end - start);
  float a3 = -2.0 / (T * T * T) * (end - start);

  static unsigned long ts[6] = {0};
  static uint16_t target[6] = {0};
  static uint16_t current[6] = {USMIN};

  float t;
  if (target[joint] != end) {
    ts[joint] = millis();
    target[joint] = end;
  }

  t = (millis() - ts[joint]) / 1000.0;

  if (t <= T) {
    current[joint] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  } else {
    complete[joint] = true;
  }

  return current[joint];
}

void initServos() {
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);
  for (int i = 0; i < 6; ++i) {
    j_pre[i] = j_now[i];
    pwm.writeMicroseconds(joints[i], j_pre[i]);
  }
}

bool allComplete() {
  for (int i = 0; i < 6; ++i)
    if (!complete[i]) return false;
  return true;
}

void prepareNextMove() {
  for (int i = 0; i < 6; ++i) {
    j_pre[i] = j_now[i];
    complete[i] = false;
  }
  if (!isQueueEmpty()) dequeue(j_now);
  tf = j_now[6];
}

void updateServos() {
  for (int i = 0; i < 6; ++i) {
    pwm.writeMicroseconds(joints[i], trajectory(j_pre[i], j_now[i], i));
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  initServos();
  j_now[6] = 150;
  delay(500);
}

void loop() {
  readSerial2Data();
  if (allComplete()) prepareNextMove();
  updateServos();
}

