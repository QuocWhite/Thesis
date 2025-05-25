#include <Servo.h>

Servo servo1;
Servo servo2;

int led = 11;

void setup() {
  Serial.begin(9600);
  servo1.attach(9);  // Connect to pin 9
  servo2.attach(10); // Connect to pin 10
  pinMode(led, OUTPUT);
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n'); // Read line
    command.trim();  // Remove any whitespace

    // Format: S1:90 or S2:45
    if (command.startsWith("S1:")) {
      int angle = command.substring(3).toInt();
      angle = constrain(angle, 0, 180);
      servo1.write(angle);
    } else if (command.startsWith("S2:")) {
      int angle = command.substring(3).toInt();
      angle = constrain(angle, 0, 180);
      servo2.write(angle);
    }
    else if ( command == "led")
    {
      digitalWrite(led, HIGH);
    }
    else
    {
      /* Do nothing */
    }
  }
}

