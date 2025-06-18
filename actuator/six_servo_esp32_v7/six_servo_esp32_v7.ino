#include "ArmControl.h"

void setup() {
  Serial.begin(115200);
  ArmControl::initialize();
}

void loop() {
  ArmControl::handleSerialCommands();
  ArmControl::updateMotion();
}

