#include <Arduino.h>


enum INA_REGISTER : uint8_t
{
  CONFIG = 0,
  ADC_CONFIG = 1,
  SHUNT_CAL = 2,    //Shunt Calibration
  SHUNT_TEMPCO = 3, //Shunt Temperature Coefficient
  VSHUNT = 4,       //Shunt Voltage Measurement 24bit
  VBUS = 5,         //Bus Voltage Measurement 24bit
  DIETEMP = 6,
  CURRENT = 7,   //Current Result 24bit
  POWER = 8,     //Power Result 24bit
  ENERGY = 9,    //Energy Result 40bit
  CHARGE = 0x0A, //Charge Result 40bit
  DIAG_ALRT = 0x0b,
  SOVL = 0x0c, //Shunt Overvoltage Threshold
  SUVL = 0x0d, //Shunt Undervoltage Threshold
  BOVL = 0x0e, //Bus Overvoltage Threshold
  BUVL = 0x0f, //Bus Undervoltage Threshold
  TEMP_LIMIT = 0x10,
  PWR_LIMIT = 0x11,
  MANUFACTURER_ID = 0xFE,
  DIE_ID = 0xFF
};

enum DIAG_ALRT_FIELD : uint16_t
{
  ALATCH = 15,
  CNVR = 14,
  SLOWALERT = 13,
  APOL = 12,
  ENERGYOF = 11,
  CHARGEOF = 10,
  MATHOF = 9,
  RESERVED = 8,
  TMPOL = 7,
  SHNTOL = 6,
  SHNTUL = 5,
  BUSOL = 4,
  BUSUL = 3,
  POL = 2,
  CNVRF = 1,
  MEMSTAT = 0
};


const uint16_t ALL_ALERT_BITS = (bit(DIAG_ALRT_FIELD::TMPOL) |
                                 bit(DIAG_ALRT_FIELD::SHNTOL) |
                                 bit(DIAG_ALRT_FIELD::SHNTUL) |
                                 bit(DIAG_ALRT_FIELD::BUSOL) |
                                 bit(DIAG_ALRT_FIELD::BUSUL) |
                                 bit(DIAG_ALRT_FIELD::POL));

void RedLED(bool value);
void GreenLED(bool value);
void CalculateLSB();