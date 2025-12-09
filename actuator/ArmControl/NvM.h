#ifndef ARM_CONTROL_NVM_H
#define ARM_CONTROL_NVM_H

#include "CommonDefines.h"
#include <EEPROM.h>

/* === Non-volatile Memory API === */
class ArmControlNvM {
public:
  ArmControlNvM();
  void initializeEEPROM(
    UI_16* eepromValues,
    const UI_16* factoryInit
  );
  void WriteDataToEEPROM(
    const UI_16 *value,
    UI_8 magic_val
  );
  void ReadJointDataFromEEPROM(
    UI_16 *value
  );
};

#endif
