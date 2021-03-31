#include "settings.h"

#include "modbuscrc.h"

void WriteConfigToEEPROM(uint8_t *settings, uint16_t size)
{
  //TODO: We should probably check EEPROM.length() to ensure its big enough

  //Start write at address 2, checksum is bytes 0 and 1
  uint16_t EEPROMaddress = 2;
  for (uint16_t i = 0; i < size; i++)
  {
    EEPROM.update(EEPROMaddress, settings[i]);
    EEPROMaddress++;
  }

  //Generate and save the checksum for the setting data block
  uint16_t checksum = ModbusRTU_CRC(settings, size);
  EEPROM.put(0, checksum);
}

bool ReadConfigFromEEPROM(uint8_t *settings, uint16_t size)
{
  uint16_t EEPROMaddress = 2;
  for (uint16_t i = 0; i < size; i++)
  {
    settings[i] = EEPROM.read(EEPROMaddress);
    EEPROMaddress++;
  }

  // Calculate the checksum
  uint16_t checksum = ModbusRTU_CRC(settings, size);

  uint16_t existingChecksum;
  EEPROM.get(0, existingChecksum);

  if (checksum == existingChecksum)
  {
    //Return TRUE
    return true;
  }

  //Original data is now corrupt so return FALSE
  return false;
}

void FactoryDefault(uint16_t size)
{
  //Clear checksum
  EEPROM.put(0, 0x0000);
}
