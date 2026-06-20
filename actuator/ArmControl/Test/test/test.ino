#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 // This is the 'minimum' pulse length count (adjust if needed)
#define SERVOMAX  600 // This is the 'maximum' pulse length count (adjust if needed)
#define NUM_SERVOS 13   // Channels 0 to 5

void setup() {
  Serial.begin(115200);
  Serial.println("PCA9685 Servo test!");

  pwm.begin();
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz update

  delay(10);
}

void loop() {
  // Sweep each servo from min to max
  for (uint8_t servo = 0; servo < NUM_SERVOS; servo++) {
    Serial.print("Servo "); Serial.print(servo); Serial.println(" to min");
    pwm.setPWM(servo, 0, SERVOMIN);
    delay(500);
  }
  delay(1000);

  for (uint8_t servo = 0; servo < NUM_SERVOS; servo++) {
    Serial.print("Servo "); Serial.print(servo); Serial.println(" to max");
    pwm.setPWM(servo, 0, SERVOMAX);
    delay(500);
  }
  delay(1000);

  for (uint8_t servo = 0; servo < NUM_SERVOS; servo++) {
    Serial.print("Servo "); Serial.print(servo); Serial.println(" to mid");
    pwm.setPWM(servo, 0, (SERVOMIN + SERVOMAX) / 2);
    delay(500);
  }
  delay(1000);
}
