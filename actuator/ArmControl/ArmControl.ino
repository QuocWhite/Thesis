#include "ArmControl.h"

ArmControl arm;

void setup() {
  arm.initialize();
}

void loop() {
  arm.handleSerialCommands();
  arm.updateMotion();
}

