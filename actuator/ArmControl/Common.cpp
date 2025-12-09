#include "Common.h"

UI_16 ConsTrain(UI_16 value) {
  if (value < SERVO_MIN_SOURCE)
    return SERVO_MIN_SOURCE;
  if (value > SERVO_MAX_SOURCE)
    return SERVO_MAX_SOURCE;
  return value;
}
/// --- End of ConsTrain() ---

UI_16 AngletoValue(
  UI_16 angle,
  UI_16 minTarget,
  UI_16 maxTarget
) {
  UI_16 convertedAngle = ConsTrain(angle);
  return (convertedAngle - SERVO_MIN_SOURCE) *
    (maxTarget - minTarget) /
    (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE) +
    minTarget;
}
/// --- End of AngletoValue() ---

UI_16 ValuetoAngle(
  UI_16 value,
  UI_16 minTarget,
  UI_16 maxTarget
) {
  return ((value - minTarget) *
    (SERVO_MAX_SOURCE - SERVO_MIN_SOURCE)) /
    (maxTarget - minTarget) + SERVO_MIN_SOURCE;
}
/// --- End of ValuetoAngle() ---
