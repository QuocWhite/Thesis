#include "Input.h"
#include "Common.h"

ArmControlInput::ArmControlInput() {}

void ArmControlInput::HandleDataInput(
  UI_16 *value,
  Trajectory *Queue,
  String rawInput
) {
  UI_16 base, right, left, twist, wrist, finger;
  base = rawInput.substring(
    FIRST_INPUT, rawInput.indexOf('r')
  ).toInt();
  right = rawInput.substring(
    rawInput.indexOf('r') + INCREASE,
    rawInput.indexOf('l')
  ).toInt();
  left = rawInput.substring(
    rawInput.indexOf('l') + INCREASE,
    rawInput.indexOf('t')
  ).toInt();
  twist = rawInput.substring(
    rawInput.indexOf('t') + INCREASE,
    rawInput.indexOf('w')
  ).toInt();
  wrist = rawInput.substring(
    rawInput.indexOf('w') + INCREASE,
    rawInput.indexOf('f')
  ).toInt();
  finger = rawInput.substring(
    rawInput.indexOf('f') + INCREASE
  ).toInt();
  value[BASE_JOINT] = AngletoValue(
    base, FACT_BASE_MIN_ANGLE, FACT_BASE_MAX_ANGLE
  );
  value[RIGHT_JOINT] = AngletoValue(
    right, FACT_RIGHT_MIN_ANGLE, FACT_RIGHT_MAX_ANGLE
  );
  value[LEFT_JOINT] = AngletoValue(
    left, FACT_LEFT_MIN_ANGLE, FACT_LEFT_MAX_ANGLE
  );
  value[TWIST_JOINT] = AngletoValue(
    twist, FACT_TWIST_MIN_ANGLE, FACT_TWIST_MAX_ANGLE
  );
  value[WRIST_JOINT] = AngletoValue(
    wrist, FACT_WRIST_MIN_ANGLE, FACT_WRIST_MAX_ANGLE
  );
  value[FINGER_JOINT] = AngletoValue(
    finger, FACT_FINGER_MIN_ANGLE, FACT_FINGER_MAX_ANGLE
  );
  ArmControlQueue queueObj; // Use main queue in your app!
  queueObj.QueueEndQueue(Queue, value);
}
/// --- End of HandleDataInput() ---

void ArmControlInput::ReadManualCalib(
  UI_16 *value
) {
  for (UI_8 count = FIRST_JOINT;
       count < MAX_JOINT;
       count++) {
    UI_8 low = Serial.read();
    UI_8 high = Serial.read();
    value[count] = (high << SHIFT_BYTE) | low;
  }
  // EEPROM handled by NvM module
}
/// --- End of ReadManualCalib() ---
