#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  
#define SERVOMAX  520 
#define USMIN  360 
#define USMID  1400
#define USMAX  2550 
#define SERVO_FREQ 50 
#define J1 12   /* Base */
#define J2 0    /* Right */
#define J3 1    /* Left */
#define J4 8    /* Twist */
#define J5 7    /* Wrist */
#define J6 5    /* Finger */
#define QUEUE_SIZE  16

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
uint8_t join[6] = {J1, J2, J3, J4, J5, J6};

uint16_t Queue_State_Array0[QUEUE_SIZE] = {0}, Queue_State_Array1[QUEUE_SIZE] = {0},
         Queue_State_Array2[QUEUE_SIZE] = {0}, Queue_State_Array3[QUEUE_SIZE] = {0},
         Queue_State_Array4[QUEUE_SIZE] = {0}, Queue_State_Array5[QUEUE_SIZE] = {0},
         Queue_State_Array6[QUEUE_SIZE] = {0};
CircularQueue_Types TQ = {
  -1, -1, QUEUE_SIZE,
  Queue_State_Array0, Queue_State_Array1, Queue_State_Array2,
  Queue_State_Array3, Queue_State_Array4, Queue_State_Array5, Queue_State_Array6
};
// our servo # counter

uint16_t j_pre_pluse[6] = {USMIN, USMIN, USMIN, USMIN, USMIN, USMIN},
         j_now_pluse[7] = {USMIN, USMIN, USMIN, USMIN, USMIN, USMIN, 150},
         data_to_CQ[7]  = {USMIN, USMIN, USMIN, USMIN, USMIN, USMIN, 150};
float tf = 150;
float max_pluse;
bool complete[6];
bool CircularQueue_IsFull(CircularQueue_Types *Queue) {
  bool Key = false;
  if ((Queue->Front == Queue->Rear + 1) || (Queue->Front == 0 && Queue->Rear == Queue->Size - 1)) {
    Key = true;
  }
  return Key;
}
bool CircularQueue_IsEmpty(CircularQueue_Types *Queue) {
  bool Key = false;
  if (Queue->Front == -1) {
    Key = true;
  }
  return Key;
}
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
void CircularQueue_PopData(CircularQueue_Types *Queue, uint16_t *OutputData){
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
    }
    else Queue->Front = (Queue->Front + 1) % Queue->Size;
  }
}
void Display(CircularQueue_Types *Queue) {
  uint8_t i;
  if (CircularQueue_IsEmpty(Queue)) {
    Serial.println("Empty");
  } else {
    Serial.printf("Front: %d\n", Queue->Front);
    Serial.print("QueArr: ");
    for (i = Queue->Front; i != Queue->Rear; i = (i + 1) % Queue->Size) {
      Serial.printf("%d ,%d ,%d , %d ,%d ,%d \n", Queue->QueArrJ0[i], Queue->QueArrJ1[i], Queue->QueArrJ2[i],
                    Queue->QueArrJ3[i], Queue->QueArrJ4[i], Queue->QueArrJ5[i]);
    }
    Serial.printf("%d ,%d ,%d , %d ,%d ,%d \n", Queue->QueArrJ0[i], Queue->QueArrJ1[i], Queue->QueArrJ2[i],
                  Queue->QueArrJ3[i], Queue->QueArrJ4[i], Queue->QueArrJ5[i]);
    Serial.printf("Rear: %d\n", Queue->Rear);
  }
}
void SerialEvent2() {
  if (Serial.available() > 0) {
    String dataInS1 = Serial.readStringUntil('\n');//pm1400m2600m3-600s0e
    if (dataInS1.startsWith("s")) {
      int t1 = (dataInS1.substring(1, dataInS1.indexOf('a'))).toInt();
      int t2 = (dataInS1.substring(dataInS1.indexOf('a') + 1, dataInS1.indexOf('b'))).toInt();
      int t3 = (dataInS1.substring(dataInS1.indexOf('b') + 1, dataInS1.indexOf('c'))).toInt();
      int t4 = (dataInS1.substring(dataInS1.indexOf('c') + 1, dataInS1.indexOf('d'))).toInt();
      int t5 = (dataInS1.substring(dataInS1.indexOf('d') + 1, dataInS1.indexOf('e'))).toInt();
      int t6 = (dataInS1.substring(dataInS1.indexOf('e') + 1, dataInS1.indexOf('f'))).toInt();
      int t7 = (dataInS1.substring(dataInS1.indexOf('f') + 1, dataInS1.length())).toInt();
          Serial.print(dataInS1);
          Serial.print(", ");
          Serial.print(CircularQueue_IsFull(&TQ));
          Serial.print(", ");
          Serial.println(CircularQueue_IsEmpty(&TQ));
      data_to_CQ[0] = map(constrain(t1+109, 0, 218), 0, 218, USMIN, USMAX);
      data_to_CQ[1] = map(constrain(172 - t2, 5, 140), 0, 218, USMIN, USMAX);
      data_to_CQ[2] = map(constrain(t3 + t2 + 37, 0, 218), 0, 218, USMIN, USMAX);
      data_to_CQ[3] = map(constrain(t4+109, 0, 218), 0, 218, USMIN, USMAX);
      data_to_CQ[4] = map(constrain(t5+99+2, 0, 198), 0, 198, USMAX, USMIN);
      data_to_CQ[5] = map(constrain(t6+5+103, 0, 206), 0, 206, USMIN, USMAX);
      data_to_CQ[6] = t7;
      CircularQueue_PushData(&TQ, data_to_CQ);
    }
  }
}
uint16_t theta(uint16_t theta_s, uint16_t theta_e, int joint)
{
  float a0, a1, a2, a3;
  static unsigned long ts[6];
  static uint16_t endt[6];
  static uint16_t thet[6];
  float t;
  float tt = (float) tf/1000; 
  if (endt[joint] != theta_e)
  {
    ts[joint] = millis();
    endt[joint] = theta_e;
  }
  a0 = theta_s;
  a1 = 0;
  a2 = 3.0 / (tt * tt) * (theta_e - theta_s);
  a3 = -2.0 / (tt * tt * tt) * (theta_e - theta_s);
  t = (float)(millis() - ts[joint]) / 1000;


  if (t <= tt)
  {

    thet[joint] = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  }
  else complete[joint] = true;

  return thet[joint];
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);  

  delay(10);

  j_pre_pluse[0] = map(0, -109, 109, USMIN, USMAX);
  j_pre_pluse[1] = map(37, 0, 218, USMIN, USMAX);
  j_pre_pluse[2] = map(127, 0, 218, USMIN, USMAX);
  j_pre_pluse[3] = map(109, 0, 218, USMIN, USMAX);
  j_pre_pluse[4] = map(11, 0, 198, USMAX, USMIN);
  j_pre_pluse[5] = map(108, 0, 206, USMIN, USMAX);
  for (int i = 0; i < 6; i++) {
    j_now_pluse[i] = j_pre_pluse[i];
  }
  j_now_pluse[6]=150;
  for (int i = 0; i < 6; i++) {
    pwm.writeMicroseconds(join[i], j_pre_pluse[i]);
  }
  delay(500);

}

void loop() {
  SerialEvent2();
  if (complete[0] && complete[1] && complete[2] && complete[3] && complete[4] && complete[5])  {
    for (int i = 0; i < 6; i++) {
      j_pre_pluse[i] = j_now_pluse[i];
      complete[i] = false;
    }
    if (!CircularQueue_IsEmpty(&TQ))  CircularQueue_PopData(&TQ, j_now_pluse);
  }
  tf = j_now_pluse[6];
  for (int i = 0; i < 6; i++) {
    pwm.writeMicroseconds(join[i], theta(j_pre_pluse[i], j_now_pluse[i], i));
  }
}
