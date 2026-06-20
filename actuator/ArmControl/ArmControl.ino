#include "ArmControl.h"

static ArmControl arm;

void setup() {
  arm.initialize();
}

void loop() {
  arm.handleSerialCommands();
  arm.updateMotion();
}
