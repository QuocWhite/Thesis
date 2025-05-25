#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_FREQ 50

// Pulse ranges
#define MG996R_MIN 500
#define MG996R_MAX 2400

#define MG90S_MIN 600
#define MG90S_MAX 2300

// Servo channel mapping
#define J1 12  // Base - MG996R
#define J2 0   // Right - MG996R
#define J3 1   // Left - MG996R
#define J4 8   // Twist - MG996R
#define J5 7   // Wrist - MG90S
#define J6 5   // Finger - MG90S

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

uint8_t join[6] = {J1, J2, J3, J4, J5, J6};

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

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("Starting servo self-test...");
  testAllServos();
  Serial.println("Servo self-test complete.");
}

void loop() {
  // Empty loop
}

