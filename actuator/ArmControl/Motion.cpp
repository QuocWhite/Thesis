#include "Motion.h"

ArmControlMotion::ArmControlMotion()
  : servoDriver(nullptr), jointChannels(nullptr) {}

void ArmControlMotion::init(
  Adafruit_PWMServoDriver* driver,
  const UI_8* channels
) {
  servoDriver = driver;
  jointChannels = channels;
}
/// --- End of init() ---

void ArmControlMotion::updateServoPositions(
  UI_16 *previousPulse,
  UI_16 *currentPulse,
  UI_8 *jointMotionComplete
) {
  for (UI_8 count = 0; count < MAX_JOINT; count++)
    servoDriver->writeMicroseconds(
      jointChannels[count],
      computeCubicTrajectory(
        previousPulse[count],
        currentPulse[count],
        count,
        jointMotionComplete
      )
    );
}
/// --- End of updateServoPositions() ---

UI_16 ArmControlMotion::computeCubicTrajectory(
  UI_16 startPos,
  UI_16 endPos,
  UI_8 jointIdx,
  UI_8 *jointMotionComplete
) {
  static UI_16 startTime[MAX_JOINT];
  static UI_16 lastTarget[MAX_JOINT];
  static UI_16 currentOutput[MAX_JOINT];
  SI_32 delta = (SI_32)endPos - (SI_32)startPos;
  if (lastTarget[jointIdx] != endPos) {
    startTime[jointIdx] = millis();
    lastTarget[jointIdx] = endPos;
  }
  UI_16 elapsedMs = millis() - startTime[jointIdx];
  SI_32 T = MOTION_DURATION;
  SI_32 t = elapsedMs;
  SI_32 a0 = startPos * CUBIC_SCALE_POS;
  SI_32 a1 = CUBIC_ZERO;
  SI_32 a2 = (CUBIC_COEFF_A2 * delta *
    CUBIC_SCALE_A2) / (T * T);
  SI_32 a3 = (CUBIC_COEFF_A3 * delta *
    CUBIC_SCALE_A3) / (T * T * T);
  SI_32 pos;
  if (t <= T) {
    pos = a0 / CUBIC_SCALE_POS +
      (a1 * t) +
      (a2 * t * t) / CUBIC_SCALE_A2 +
      (a3 * t * t * t) / CUBIC_SCALE_A3;
    currentOutput[jointIdx] = (UI_16)pos;
  } else {
    jointMotionComplete[jointIdx] = D_TRUE;
    currentOutput[jointIdx] = endPos;
  }
  return currentOutput[jointIdx];
}
/// --- End of computeCubicTrajectory() ---

void ArmControlMotion::testAllServos(
  const UI_8* jointChannels
) {
  UI_16 pulseMin = COMMON_MIN_ANGLE;
  UI_16 pulseMid = (COMMON_MIN_ANGLE +
    COMMON_MAX_ANGLE) / 2;
  for (UI_8 count = 0; count < MAX_JOINT; count++) {
    servoDriver->writeMicroseconds(
      jointChannels[count], pulseMin
    );
    delay(DELAY_TIME);
    servoDriver->writeMicroseconds(
      jointChannels[count], pulseMid
    );
    delay(DELAY_TIME);
  }
}
/// --- End of testAllServos() ---
