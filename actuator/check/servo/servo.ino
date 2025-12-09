#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_PWM_MIN           360
#define SERVO_PWM_MAX           2550
#define JOINT                   4
#define SERVO_FREQUENCY         50
#define OCLFRQ                  27000000

Adafruit_PWMServoDriver servoDriver = Adafruit_PWMServoDriver();

void setup(){
  Serial.begin(115200);
  servoDriver.begin();
  servoDriver.setOscillatorFrequency(OCLFRQ);
  servoDriver.setPWMFreq(SERVO_FREQUENCY);
}

void loop(){
  int count;
  for (count = SERVO_PWM_MIN; count < SERVO_PWM_MAX; count++) {
    servoDriver.writeMicroseconds(JOINT, count);
    delay(20);
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') { // Enter pressed
        Serial.print("Current value written to servo: ");
        Serial.println(count);
        break; // Exit the for loop
      }
    }
  }
  while (Serial.available()) Serial.read(); // Clear serial buffer
  delay(500); // Prevent spamming
}
