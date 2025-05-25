#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Constants and definitions
#define SERVOMAX  520
#define USMIN  360 
#define USMID  1400
#define USMAX  2550 
#define SERVO_FREQ 50 
#define QUEUE_SIZE  16
#define MG996R_MIN 500
#define MG996R_MAX 2400
#define MG90S_MIN 600
#define MG90S_MAX 2300

// Servo channel mapping
#define J1 12   // Base
#define J2 0    // Right
#define J3 1    // Left
#define J4 8    // Twist
#define J5 7    // Wrist
#define J6 5    // Finger

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Joint channel array
uint8_t join[6] = {J1, J2, J3, J4, J5, J6};

// Queue structure definition
typedef struct {
  int8_t Front, Rear, Size;
  uint16_t *QueArrJ0;
  uint16_t *QueArrJ1;
  uint16_t *QueArrJ2;
  uint16_t *QueArrJ3;
  uint16_t *QueArrJ4;
  uint16_t *QueArrJ5;
  uint16_t *QueArrJ6;
} CircularQueue_Types;

// Queue storage arrays
uint16_t Queue_State_Array0[QUEUE_SIZE] = {0}, Queue_State_Array1[QUEUE_SIZE] = {0},
         Queue_State_Array2[QUEUE_SIZE] = {0}, Queue_State_Array3[QUEUE_SIZE] = {0},
         Queue_State_Array4[QUEUE_SIZE] = {0}, Queue_State_Array5[QUEUE_SIZE] = {0},
         Queue_State_Array6[QUEUE_SIZE] = {0};

// Queue instance
CircularQueue_Types TQ = {
  -1, -1, QUEUE_SIZE,
  Queue_State_Array0, Queue_State_Array1, Queue_State_Array2,
  Queue_State_Array3, Queue_State_Array4, Queue_State_Array5, Queue_State_Array6
};

// Position arrays for trajectory
uint16_t j_pre_pluse[6] = {USMIN, USMIN, USMIN, USMIN, USMIN, USMIN},
         j_now_pluse[7] = {USMIN, USMIN, USMIN, USMIN, USMIN, USMIN, 150},
         data_to_CQ[7]  = {USMIN, USMIN, USMIN, USMIN, USMIN, USMIN, 150};

// Motion duration and flags
float tf = 150;
bool complete[6];

// Queue utility: check if full
bool CircularQueue_IsFull(CircularQueue_Types *Queue) {
  return (Queue->Front == Queue->Rear + 1) || (Queue->Front == 0 && Queue->Rear == Queue->Size - 1);
}

// Queue utility: check if empty
bool CircularQueue_IsEmpty(CircularQueue_Types *Queue) {
  return Queue->Front == -1;
}

// Push joint position data into queue
void CircularQueue_PushData(CircularQueue_Types *Queue, uint16_t *InputData) {
  if (!CircularQueue_IsFull(Queue)) {
    if (Queue->Front == -1) {
      Queue->Front = 0;
    }
    Queue->Rear = (Queue->Rear + 1) % Queue->Size;
    Queue->QueArrJ0[Queue->Rear] = InputData[0];
    Queue->QueArrJ1[Queue->Rear] = InputData[1];
    Queue->QueArrJ2[Queue->Rear] = InputData[2];
    Queue->QueArrJ3[Queue->Rear] = InputData[3];
    Queue->QueArrJ4[Queue->Rear] = InputData[4];
    Queue->QueArrJ5[Queue->Rear] = InputData[5];
    Queue->QueArrJ6[Queue->Rear] = InputData[6];
  }
}

void moveServoHalfway(uint8_t channel) {
  int us_min, us_mid;

  // Set pulse range based on servo type
  if (channel == J5 || channel == J6) {
    us_min = MG90S_MIN;
    us_mid = (MG90S_MIN + MG90S_MAX) / 2;
  } else {
    us_min = MG996R_MIN;
    us_mid = (MG996R_MIN + MG996R_MAX) / 2;
  }

  Serial.print("Testing Servo Channel: ");
  Serial.println(channel);

  pwm.writeMicroseconds(channel, us_min);
  delay(500);
  pwm.writeMicroseconds(channel, us_mid);
  delay(500);
}

void testAllServos() {
  for (int i = 0; i < 6; i++) {
    moveServoHalfway(join[i]);
  }
}

// Pop joint position data from queue
void CircularQueue_PopData(CircularQueue_Types *Queue, uint16_t *OutputData) {
  if (!CircularQueue_IsEmpty(Queue)) {
    OutputData[0] = Queue->QueArrJ0[Queue->Front];
    OutputData[1] = Queue->QueArrJ1[Queue->Front];
    OutputData[2] = Queue->QueArrJ2[Queue->Front];
    OutputData[3] = Queue->QueArrJ3[Queue->Front];
    OutputData[4] = Queue->QueArrJ4[Queue->Front];
    OutputData[5] = Queue->QueArrJ5[Queue->Front];
    OutputData[6] = Queue->QueArrJ6[Queue->Front];

    if (Queue->Front == Queue->Rear) {
      Queue->Front = -1;
      Queue->Rear = -1;
    } else {
      Queue->Front = (Queue->Front + 1) % Queue->Size;
    }
  }
}

// Display contents of queue (for debugging)
void Display(CircularQueue_Types *Queue) {
  uint8_t i;
  if (CircularQueue_IsEmpty(Queue)) {
    Serial.println("Empty");
  } else {
    Serial.printf("Front: %d\n", Queue->Front);
    Serial.print("QueArr: \n");
    for (i = Queue->Front; i != Queue->Rear; i = (i + 1) % Queue->Size) {
      Serial.printf("%d ,%d ,%d , %d ,%d ,%d \n", Queue->QueArrJ0[i], Queue->QueArrJ1[i], Queue->QueArrJ2[i],
                    Queue->QueArrJ3[i], Queue->QueArrJ4[i], Queue->QueArrJ5[i]);
    }
    Serial.printf("%d ,%d ,%d , %d ,%d ,%d \n", Queue->QueArrJ0[i], Queue->QueArrJ1[i], Queue->QueArrJ2[i],
                  Queue->QueArrJ3[i], Queue->QueArrJ4[i], Queue->QueArrJ5[i]);
    Serial.printf("Rear: %d\n", Queue->Rear);
  }
}

// Read serial data and queue it if valid
void readSerial2Data() {
  if (Serial.available() > 0) {
    String dataInS1 = Serial.readStringUntil('\n');
    if (dataInS1.startsWith("s")) {
      int t1 = (dataInS1.substring(1, dataInS1.indexOf('a'))).toInt();
      int t2 = (dataInS1.substring(dataInS1.indexOf('a') + 1, dataInS1.indexOf('b'))).toInt();
      int t3 = (dataInS1.substring(dataInS1.indexOf('b') + 1, dataInS1.indexOf('c'))).toInt();
      int t4 = (dataInS1.substring(dataInS1.indexOf('c') + 1, dataInS1.indexOf('d'))).toInt();
      int t5 = (dataInS1.substring(dataInS1.indexOf('d') + 1, dataInS1.indexOf('e'))).toInt();
      int t6 = (dataInS1.substring(dataInS1.indexOf('e') + 1, dataInS1.indexOf('f'))).toInt();
      int t7 = (dataInS1.substring(dataInS1.indexOf('f') + 1)).toInt();

      // Map and constrain incoming data
      data_to_CQ[0] = map(constrain(t1 + 109, 0, 218), 0, 218, USMIN, USMAX);
      data_to_CQ[1] = map(constrain(172 - t2, 5, 140), 0, 218, USMIN, USMAX);
      data_to_CQ[2] = map(constrain(t3 + t2 + 37, 0, 218), 0, 218, USMIN, USMAX);
      data_to_CQ[3] = map(constrain(t4 + 109, 0, 218), 0, 218, USMIN, USMAX);
      data_to_CQ[4] = map(constrain(t5 + 101, 0, 198), 0, 198, USMAX, USMIN);
      data_to_CQ[5] = map(constrain(t6 + 108, 0, 206), 0, 206, USMIN, USMAX);
      data_to_CQ[6] = t7;

      // Push parsed data into queue
      CircularQueue_PushData(&TQ, data_to_CQ);
    }
  }
}

// Check if all joints completed their move
bool allComplete() {
  for (int i = 0; i < 6; i++) {
    if (!complete[i]) return false;
  }
  return true;
}

// Prepare for next set of movements
void prepareNextMove() {
  for (int i = 0; i < 6; i++) {
    j_pre_pluse[i] = j_now_pluse[i];
    complete[i] = false;
  }
  if (!CircularQueue_IsEmpty(&TQ)) {
    CircularQueue_PopData(&TQ, j_now_pluse);
  }
}

// Calculate cubic trajectory for servo motion
uint16_t theta(uint16_t theta_s, uint16_t theta_e, int joint) {
  float a0, a1, a2, a3;
  static unsigned long ts[6];
  static uint16_t endt[6];
  static uint16_t thet[6];
  float t;
  float tt = (float) tf / 1000;

  if (endt[joint] != theta_e) {
    ts[joint] = millis();
    endt[joint] = theta_e;
  }

  a0 = theta_s;
  a1 = 0;
  a2 = 3.0 / (tt * tt) * (theta_e - theta_s);
  a3 = -2.0 / (tt * tt * tt) * (theta_e - theta_s);

  t = (float)(millis() - ts[joint]) / 1000;

  if (t <= tt) {
    thet[joint] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  } else {
    complete[joint] = true;
  }

  return thet[joint];
}

// Send current position command to servos
void updateServos() {
  for (int i = 0; i < 6; i++) {
    pwm.writeMicroseconds(join[i], theta(j_pre_pluse[i], j_now_pluse[i], i));
  }
}

void init_state()
{
  // Initialize joint start positions
  j_pre_pluse[0] = map(0, -109, 109, USMIN, USMAX);
  j_pre_pluse[1] = map(37, 0, 218, USMIN, USMAX);
  j_pre_pluse[2] = map(127, 0, 218, USMIN, USMAX);
  j_pre_pluse[3] = map(109, 0, 218, USMIN, USMAX);
  j_pre_pluse[4] = map(11, 0, 198, USMAX, USMIN);
  j_pre_pluse[5] = map(108, 0, 206, USMIN, USMAX);

  for (int i = 0; i < 6; i++) {
    j_now_pluse[i] = j_pre_pluse[i];
    pwm.writeMicroseconds(join[i], j_pre_pluse[i]);
  }
  j_now_pluse[6] = 150;
}

// Arduino setup function
void setup() {
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("Starting servo self-test...");
  testAllServos();
  Serial.println("Servo self-test complete.");

  init_state();

  if (allComplete()) {
    prepareNextMove();
  }
  Display(&TQ);

  delay(500);
}

// Arduino loop function
void loop() {
  readSerial2Data();

  if (allComplete()) {
    prepareNextMove();
  }

  tf = j_now_pluse[6];  // Update time frame for interpolation
  updateServos();
}

