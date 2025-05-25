#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define J1 12        // Base servo channel
#define USMIN 360    // Minimum pulse width
#define USMAX 2550   // Maximum pulse width
#define SERVO_FREQ 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("Testing base servo on channel 12");

  // Move to min position
  pwm.writeMicroseconds(J1, USMIN);
  Serial.println("Moved to min position");
  delay(1000);

  // Move to mid position
  pwm.writeMicroseconds(J1, (USMIN + USMAX) / 2);
  Serial.println("Moved to mid position");
  delay(1000);

  // Move to max position
  pwm.writeMicroseconds(J1, USMAX);
  Serial.println("Moved to max position");
  delay(1000);
}

void loop() {
  // Sweep back and forth
  for (int pos = USMIN; pos <= USMAX; pos += 10) {
    pwm.writeMicroseconds(J1, pos);
    delay(15);
  }
  for (int pos = USMAX; pos >= USMIN; pos -= 10) {
    pwm.writeMicroseconds(J1, pos);
    delay(15);
  }
}

