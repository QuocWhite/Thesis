#ifndef ARM_CONTROL_COMMON_H
#define ARM_CONTROL_COMMON_H

#include "CommonDefines.h"

/* === Common Utility API === */
UI_16 ConsTrain(UI_16 value);
UI_16 AngletoValue(
  UI_16 angle,
  UI_16 minTarget,
  UI_16 maxTarget
);
UI_16 ValuetoAngle(
  UI_16 value,
  UI_16 minTarget,
  UI_16 maxTarget
);

#endif
