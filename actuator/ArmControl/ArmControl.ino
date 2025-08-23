#include "ArmControl.h"

void setup() {
  ArmControl::initialize();
}

void loop() {
  ArmControl::handleSerialCommands();
  ArmControl::updateMotion();
}

