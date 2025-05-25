#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVO_MIN 120   // min pulse length out of 4096
#define SERVO_MAX 600   // max pulse length out of 4096

int servoChannel = 5; // PCA9685 channel where the servo is connected

void setup() {
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);  // 50 Hz for analog servos
  Wire.setClock(400000);  // Optional: faster I2C speed
  delay(10);
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // read until newline
    int angle = input.toInt();
    angle = constrain(angle, 0, 180);
    int pulse = map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
    pwm.setPWM(servoChannel, 0, pulse);
    Serial.print("Moved to angle: ");
    Serial.println(angle);
  }
}

