#include "NvM.h"

ArmControlNvM::ArmControlNvM() {}

void ArmControlNvM::initializeEEPROM(
  UI_16* eepromValues,
  const UI_16* factoryInit
) {
  if (!EEPROM.begin(EEPROM_SIZE))
    return;
  UI_8 eepVal = EEPROM.read(MAGIC_ADDR);
  if ((eepVal != MAGIC_INIT_VAL) &&
      (eepVal != MAGIC_MODF_VAL)) {
    WriteDataToEEPROM(factoryInit, MAGIC_INIT_VAL);
  }
  ReadJointDataFromEEPROM(eepromValues);
}
/// --- End of initializeEEPROM() ---

void ArmControlNvM::WriteDataToEEPROM(
  const UI_16 *value,
  UI_8 magic_val
) {
  for (UI_8 count = FIRST_JOINT;
       count < MAX_JOINT;
       count++) {
    EEPROM.write(
      START_JOINT_ADDR + count * JOINT_BYTE_SIZE +
        MSB_OFFSET,
      (value[count] >> SHIFT_BYTE) & BYTE_MASK
    );
    EEPROM.write(
      START_JOINT_ADDR + count * JOINT_BYTE_SIZE +
        LSB_OFFSET,
      value[count] & BYTE_MASK
    );
  }
  EEPROM.write(MAGIC_ADDR, magic_val);
  EEPROM.commit();
}
/// --- End of WriteDataToEEPROM() ---

void ArmControlNvM::ReadJointDataFromEEPROM(
  UI_16 *value
) {
  for (UI_8 count = FIRST_JOINT;
       count < MAX_JOINT;
       count++) {
    UI_8 msb = EEPROM.read(
      START_JOINT_ADDR +
        count * JOINT_BYTE_SIZE +
        MSB_OFFSET
    );
    UI_8 lsb = EEPROM.read(
      START_JOINT_ADDR +
        count * JOINT_BYTE_SIZE +
        LSB_OFFSET
    );
    value[count] = ((msb << SHIFT_BYTE) | lsb);
  }
}
/// --- End of ReadJointDataFromEEPROM() ---
