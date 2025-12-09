#ifndef ARM_CONTROL_INPUT_H
#define ARM_CONTROL_INPUT_H

#include "CommonDefines.h"
#include "Queue.h"

/* === Input/Calibration API === */
class ArmControlInput {
public:
  ArmControlInput();
  void HandleDataInput(
    UI_16 *value,
    Trajectory *Queue,
    String rawInput
  );
  void ReadManualCalib(UI_16 *value);
  // Add auto calibration, trigger pin, etc as needed
};

#endif
