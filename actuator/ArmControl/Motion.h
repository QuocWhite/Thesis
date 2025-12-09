#ifndef ARM_CONTROL_MOTION_H
#define ARM_CONTROL_MOTION_H

#include "CommonDefines.h"
#include <Adafruit_PWMServoDriver.h>

/* === Motion Processing API === */
class ArmControlMotion {
public:
  ArmControlMotion();
  void init(
    Adafruit_PWMServoDriver* driver,
    const UI_8* jointChannels
  );
  void updateServoPositions(
    UI_16 *previousPulse,
    UI_16 *currentPulse,
    UI_8 *jointMotionComplete
  );
  UI_16 computeCubicTrajectory(
    UI_16 startPos,
    UI_16 endPos,
    UI_8 jointIdx,
    UI_8 *jointMotionComplete
  );
  void testAllServos(const UI_8* jointChannels);

private:
  Adafruit_PWMServoDriver* servoDriver;
  const UI_8* jointChannels;
};

#endif
