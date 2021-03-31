/*
 ____  ____  _  _  ____  __  __  ___    _  _  __
(  _ \(_  _)( \/ )(  _ \(  \/  )/ __)  ( \/ )/. |
 )(_) )_)(_  \  /  ) _ < )    ( \__ \   \  /(_  _)
(____/(____) (__) (____/(_/\/\_)(___/    \/   (_)

DIYBMS V4.0
CURRENT & VOLTAGE MONITORING SYSTEM BASEDO ON TEXAS INST. INA228

(c)2021 Stuart Pittaway

COMPILE THIS CODE USING PLATFORM.IO

LICENSE
Attribution-NonCommercial-ShareAlike 2.0 UK: England & Wales (CC BY-NC-SA 2.0 UK)
https://creativecommons.org/licenses/by-nc-sa/2.0/uk/

* Non-Commercial — You may not use the material for commercial purposes.
* Attribution — You must give appropriate credit, provide a link to the license, and indicate if changes were made.
  You may do so in any reasonable manner, but not in any way that suggests the licensor endorses you or your use.
* ShareAlike — If you remix, transform, or build upon the material, you must distribute your
  contributions under the same license as the original.
* No additional restrictions — You may not apply legal terms or technological measures
  that legally restrict others from doing anything the license permits.

** COMMERCIAL USE AND RESALE PROHIBITED **

*/

// ATTINY1614 (tinyAVR® 1-series of microcontrollers)
// http://ww1.microchip.com/downloads/en/DeviceDoc/ATtiny1614-data-sheet-40001995A.pdf

// MODBUS Protocol
// https://www.ni.com/en-gb/innovations/white-papers/14/the-modbus-protocol-in-depth.html

#if !defined(MODBUSDEFAULTBAUDRATE)
#error MODBUSDEFAULTBAUDRATE must be defined
#endif
#if !defined(MODBUSBASEADDRESS)
#error MODBUSBASEADDRESS must be defined
#endif
#if !defined(MODBUSSERIALCONFIG)
#error MODBUSSERIALCONFIG must be defined
#endif

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <EEPROM.h>

#include "main.h"
#include "settings.h"

#include "modbuscrc.h"

#include "EmbeddedFiles_Defines.h"

#define GREENLED_PIN_BITMAP PIN7_bm
#define REDLED_PIN_BITMAP PIN6_bm
#define RELAY_PIN_BITMAP PIN5_bm

//150A@50mV shunt =   122.88A @ 40.96mV (full scale ADC)
uint16_t shunt_max_current = 150;
uint16_t shunt_millivolt = 50;
//Scale the full current based on 40.96mV (max range for 20bit ADC)
//const double full_scale_current = shunt_max_current;

double full_scale_adc = 40.96;
double full_scale_current = ((double)shunt_max_current / (double)shunt_millivolt) * full_scale_adc;

double CURRENT_LSB = full_scale_current / (double)0x80000;
double RSHUNT = ((double)shunt_millivolt / 1000.0) / (double)shunt_max_current;

const double CoulombsToAmpHours = 0.00027778;

const uint8_t INA228_I2C_Address{B1000000};

uint16_t ModBusBaudRate = MODBUSDEFAULTBAUDRATE;

uint8_t ModbusSlaveAddress = MODBUSBASEADDRESS;

//Buffer to hold 8 byte modbus RTU request
uint8_t modbus[8];

uint8_t sendbuff[128];

bool relay_state = false;

struct eeprom_regs
{

  //Holds what alert events trigger the relay to turn on/high
  //uses the same values/mapping as enum DIAG_ALRT_FIELD
  uint16_t relay_trigger_bitmap = 0;
};

eeprom_regs registers;

double amphour_in = 0;
double amphour_out = 0;

volatile bool ALERT_TRIGGERED = false;
uint16_t alert = 0;

volatile bool wdt_triggered = false;
volatile uint16_t wdt_triggered_count;

void ConfigurePorts()
{

  // PA1 = SDA
  // PA2 = SCL
  // PA3 = Baud Rate
  // PA4 = Modbus Address
  // PA5 = RELAY OUTPUT
  // PA6 = RED LED
  // PA7 = GREEN LED

  // Set pins as Outputs (other pins are inputs)
  PORTA.DIR = GREENLED_PIN_BITMAP | REDLED_PIN_BITMAP | RELAY_PIN_BITMAP;

  //Relay off by default
  PORTA.OUTCLR = RELAY_PIN_BITMAP;

  // Set Port B digital outputs
  // PB0 = XDIR (RS485 transmit enable)
  // PB1 = ALERT
  // PB2 = TX (has to be set as output on tiny1614)
  // PB3 = RX
  // Set pins as Outputs (other pins are inputs)
  PORTB.DIR = PIN0_bm | PIN2_bm;
  PORTB.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_enum::PORT_ISC_BOTHEDGES_gc;

  PORTB.OUTSET = PIN2_bm; // TX is high

  USART0.CTRLB = USART_RXEN_bm | USART_TXEN_bm; //enable rx and tx
  //Enable interrupts
  sei();
}

// Read the jumper pins
void ReadJumperPins()
{
  //Switch pull ups on for the pins, if they are jumpered they are pulled to ground
  PORTA.PIN3CTRL = PORT_PULLUPEN_bm;
  PORTA.PIN4CTRL = PORT_PULLUPEN_bm;

  // PA3 = Baud Rate
  // PA4 = Modbus Address

  //High value means unconnected/not jumpered
  bool BaudRateJumper = (PORTA.IN & PIN3_bm);
  bool AddressJumper = (PORTA.IN & PIN4_bm);

  //TODO: Do something with the above jumper settings

  if (BaudRateJumper == false)
  {
    //Half the default baud rate if the jumper is connected, so 19200 goes to 9600
    ModBusBaudRate = ModBusBaudRate / 2;
  }

  if (AddressJumper == false)
  {
    //Its connected
    ModbusSlaveAddress += 8;
  }

  //Switch off pull ups
  PORTA.PIN3CTRL = 0;
  PORTA.PIN4CTRL = 0;
}

void RedLED(bool value)
{
  if (value)
  {
    PORTA.OUTSET = REDLED_PIN_BITMAP;
  }
  else
  {
    PORTA.OUTCLR = REDLED_PIN_BITMAP;
  }
}

void GreenLED(bool value)
{
  if (value)
  {
    PORTA.OUTSET = GREENLED_PIN_BITMAP;
  }
  else
  {
    PORTA.OUTCLR = GREENLED_PIN_BITMAP;
  }
}

void EnableWatchdog()
{
  wdt_triggered = false;

  // Enter protection mode
  CCP = 0xD8;

  //8 seconds
  WDT.CTRLA = WDT_PERIOD_enum::WDT_PERIOD_8KCLK_gc;

  wdt_reset();
}

void DisableSerial0TX()
{
  //On tiny1614 this saves about 10mA of current
  USART0.CTRLB &= ~(USART_TXEN_bm); /* Transmitter Enable bit mask. */
}

void EnableSerial0TX()
{
  //When the transmitter is disabled, it will no longer override the TXD pin, and the pin
  //direction is automatically set as input by hardware, even if it was configured as output by the user
  //PB2 as OUTPUT
  PORTB.DIRSET = PIN2_bm;
  USART0.CTRLB |= USART_TXEN_bm; /* Transmitter Enable bit mask. */
}

void WatchdogTriggered()
{
  // This is the watchdog timer - something went wrong and no serial activity received in over 8 seconds
  wdt_triggered = true;
  wdt_triggered_count++;
}

bool i2c_writeword(const uint8_t inareg, const uint16_t data)
{
  Wire.beginTransmission(INA228_I2C_Address);
  Wire.write(inareg);
  Wire.write((uint8_t)(data >> 8)); // Write the first (MSB) byte
  Wire.write((uint8_t)data);        // and then the second byte
  uint8_t result = Wire.endTransmission();

  //Delay after making a write to INA chip
  delayMicroseconds(10);

  return result == 0;
}

int16_t i2c_readword(const uint8_t inareg)
{
  Wire.beginTransmission(INA228_I2C_Address);
  Wire.write(inareg);
  Wire.endTransmission();
  delayMicroseconds(10);
  Wire.requestFrom(INA228_I2C_Address, (uint8_t)2); // Request 2 bytes

  uint8_t a, b;
  a = Wire.read();
  b = Wire.read();

  return ((uint16_t)a << 8) | b;
}

uint32_t i2c_readUint24(const uint8_t inareg)
{
  Wire.beginTransmission(INA228_I2C_Address);
  Wire.write(inareg);
  Wire.endTransmission();
  delayMicroseconds(10);
  Wire.requestFrom(INA228_I2C_Address, (uint8_t)3); // Request 3 bytes

  uint8_t a, b, c;
  a = Wire.read();
  b = Wire.read();
  c = Wire.read();

  uint32_t reply = (uint32_t)a << 16;
  reply += (uint32_t)b << 8;
  reply += (uint32_t)c;

  return reply;
}

uint64_t i2c_readUint40(const uint8_t inareg)
{
  Wire.beginTransmission(INA228_I2C_Address);
  Wire.write(inareg);
  Wire.endTransmission();
  delayMicroseconds(10);
  Wire.requestFrom(INA228_I2C_Address, (uint8_t)5); // Request 5 bytes

  uint8_t a, b, c, d, e;
  a = Wire.read();
  b = Wire.read();
  c = Wire.read();
  d = Wire.read();
  e = Wire.read();

  uint64_t reply = (uint64_t)a << 32;
  reply += (uint64_t)b << 24;
  reply += (uint64_t)c << 16;
  reply += (uint64_t)d << 8;
  reply += (uint64_t)e;

  return reply;
}

int64_t i2c_readInt40(const uint8_t inareg)
{
  Wire.beginTransmission(INA228_I2C_Address);
  Wire.write(inareg);
  Wire.endTransmission();
  delayMicroseconds(10);
  Wire.requestFrom(INA228_I2C_Address, (uint8_t)5); // Request 5 bytes

  uint8_t a, b, c, d, e;
  a = Wire.read();
  b = Wire.read();
  c = Wire.read();
  d = Wire.read();
  e = Wire.read();

  //Check if a twos compliment negative number
  uint64_t reply = (a & 0x80) ? (uint64_t)0xFFFFFF0000000000 : 0;

  reply += (uint64_t)a << 32;
  reply += (uint64_t)b << 24;
  reply += (uint64_t)c << 16;
  reply += (uint64_t)d << 8;
  reply += (uint64_t)e;

  //Cast to signed integer
  return (int64_t)reply;
}

//Read a 24 bit (3 byte) TWOS COMPLIMENT integer
int32_t i2c_readInt24(const uint8_t inareg)
{
  uint32_t value = (i2c_readUint24(inareg) & (uint32_t)0xFFFFF0) >> 4;

  //Is the signed bit set (bit 20)
  if (value & 0x80000UL)
  {
    //Set bits 24 to 32 to indicate negative number (twos compliment)
    value |= 0xfff00000UL;
  }

  return (int32_t)value;
}

void i2c_error()
{
  //Halt here with an error, i2c comms with INA228 failed
  for (size_t i = 0; i < 100; i++)
  {
    wdt_reset();
    RedLED(true);
    delay(100);
    RedLED(false);
    delay(100);
  }

  //Finally just hang - this will trigger the watchdog causing a reboot
  delay(10000);
}

void ResetChargeEnergyRegisters()
{
  const uint16_t value = _BV(4) | _BV(14); //RSTACC & ADCRANGE

  if (!i2c_writeword(INA_REGISTER::CONFIG, value))
  {
    i2c_error();
  }
}

volatile uint16_t diag_alrt_value = 0;

void ConfigureI2C()
{
  // join i2c bus (address optional for master)
  Wire.begin();
  //Change TWI pins to use PA1/PA2 and not PB1/PB0
  Wire.swap(1);
  //Use fast i2c
  Wire.setClock(400000);

  //See if the device is connected/soldered on board
  Wire.beginTransmission(INA228_I2C_Address); // transmit to device
  if (Wire.endTransmission() > 0)
  {
    i2c_error();
  }

  const uint16_t config_value = _BV(15) | _BV(14) | _BV(4); //RESET, RSTACC & Enable ADCRANGE = 40.96mV scale

  //Now we know the INA228 is connected, reset to defaults
  if (!i2c_writeword(INA_REGISTER::CONFIG, config_value))
  {
    i2c_error();
  }

  //Get the INA chip model number (make sure we are dealing with an INA228)

  //Clear lower 4 bits, holds chip revision (zero in my case)
  int16_t dieid = i2c_readword(INA_REGISTER::DIE_ID);
  dieid = (dieid & 0xFFF0) >> 4;
  //INA228 chip
  if (dieid != 0x228)
  {
    i2c_error();
  }

  //Conversion times for voltage and current = 2074us
  //temperature = 540us
  //256 times sample averaging

  // 1111 = Continuous bus, shunt voltage and temperature
  // 110 = 6h = 2074 µs BUS VOLT
  // 110 = 6h = 2074 µs CURRENT
  // 100 = 4h = 540 µs TEMPERATURE
  // 101 = 5h = 256 ADC sample averaging count
  // B1111 110 110 100 101
  //                   AVG
  const uint16_t adc_config_value = 0xFDA5;

  if (!i2c_writeword(INA_REGISTER::ADC_CONFIG, adc_config_value))
  {
    i2c_error();
  }

  // SHUNT_CAL = 13107.2x10^6  x CURRENT_LSB x RSHUNT
  // SHUNT_CAL must be multiplied by 4 for ADCRANGE = 1
  uint16_t shunt_cal_value = (uint16_t)((double)13107200000.0 * CURRENT_LSB * RSHUNT * (double)4.0);

  //Top 2 bits are not used
  shunt_cal_value = shunt_cal_value & 0x3FFF;

  if (!i2c_writeword(INA_REGISTER::SHUNT_CAL, shunt_cal_value))
  {
    i2c_error();
  }

  //SLOWALERT ALATCH
  diag_alrt_value = bit(DIAG_ALRT_FIELD::SLOWALERT); // | bit(DIAG_ALRT_FIELD::ALATCH);
  if (!i2c_writeword(INA_REGISTER::DIAG_ALRT, diag_alrt_value))
  {
    i2c_error();
  }

  //Check MEMSTAT=1 which proves the INA chip is not corrupt
  diag_alrt_value = i2c_readword(INA_REGISTER::DIAG_ALRT);
  if (diag_alrt_value & bit(DIAG_ALRT_FIELD::MEMSTAT) == 0)
  {
    //MEMSTAT
    //This bit is set to 0 if a checksum error is detected in the device trim memory space.
    //0h = Memory Checksum Error
    //1h = Normal Operation
    i2c_error();
  }

  //Sets the threshold for comparison of the value to detect Bus Overvoltage (overvoltage protection).
  //Unsigned representation, positive value only. Conversion factor: 3.125 mV/LSB.
  uint16_t BusOvervoltageThreshold = (uint16_t)(12.0F / 0.003125F) & 0x7FFFU;
  i2c_writeword(INA_REGISTER::BOVL, BusOvervoltageThreshold);

  //5Volt
  //uint16_t BusUndervoltageThreshold = (uint16_t)(5.0F / 0.003125F) & 0x7FFFU;
  //i2c_writeword(INA_REGISTER::BUVL, BusUndervoltageThreshold);

  //Sets the threshold for comparison of the value to detect power overlimit measurements. Unsigned representation, positive value only.
  //The value entered in this field compares directly against the value from the POWER register to determine if an
  //over power condition exists. Conversion factor: 256 × Power LSB.
  //uint16_t PowerOverLimitThreshold = (uint16_t)(100);
  //i2c_writeword(INA_REGISTER::PWR_LIMIT, PowerOverLimitThreshold);

  //Current limit based on mV scale on shunt
  //Sets the threshold for comparison of the value to detect Shunt Overvoltage (overcurrent protection). Two's complement value.
  //Conversion Factor: 1.25 µV/LSB when ADCRANGE = 1.

  //Alert over current at 0.725A
  const double x = (1.0 / full_scale_current) * full_scale_adc;
  int16_t CurrentOverThreshold = (x * 1000.0 / 1.25);
  i2c_writeword(INA_REGISTER::SOVL, CurrentOverThreshold);

  //Negative (limit current whilst charging)
  const double y = (-0.500 / full_scale_current) * full_scale_adc;
  int16_t CurrentUnderThreshold = (y * 1000.0 / 1.25);
  i2c_writeword(INA_REGISTER::SUVL, CurrentUnderThreshold);
}

void setup()
{
  //Did we have a watchdog reboot?
  if (RSTCTRL.RSTFR & RSTCTRL_WDRF_bm)
  {
    // Must be first line of code
    WatchdogTriggered();
  }
  else
  {
    wdt_triggered_count = 0;
  }

  //By default, trigger relay on all alerts
  registers.relay_trigger_bitmap = bit(DIAG_ALRT_FIELD::TMPOL) |
                         bit(DIAG_ALRT_FIELD::SHNTOL) |
                         bit(DIAG_ALRT_FIELD::SHNTUL) |
                         bit(DIAG_ALRT_FIELD::BUSOL) |
                         bit(DIAG_ALRT_FIELD::BUSUL) |
                         bit(DIAG_ALRT_FIELD::POL);

  ConfigurePorts();
  EnableWatchdog();

  for (size_t i = 0; i < 5; i++)
  {
    GreenLED(true);
    if (wdt_triggered)
    {
      RedLED(true);
    }
    delay(50);
    GreenLED(false);
    if (wdt_triggered)
    {
      RedLED(false);
    }
    delay(150);
  }

  //ReadConfigFromEEPROM
  //ReadJumperPins();

  //Disable RS485 receiver (debug!)
  PORTB.OUTSET = PIN0_bm;
  PORTB.PIN0CTRL = 0;

  ConfigureI2C();

  //Serial uses PB2/PB3 and PB0 for XDIR
  Serial.begin(ModBusBaudRate, MODBUSSERIALCONFIG);

  //0x01= Enables RS-485 mode with control of an external line driver through a dedicated Transmit Enable (TE) pin.
  //USART0.CTRLA |= B00000001;
}

double BusVoltage()
{
  //Bus voltage output. Two's complement value, however always positive.  Value in bits 23 to 4
  //195.3125uV per LSB
  return (double)195.3125 * (double)i2c_readInt24(INA_REGISTER::VBUS) / 1000000.0;
}

double ShuntVoltage()
{
  //Shunt voltage in MILLIVOLTS mV (Two's complement value)
  //78.125 nV/LSB
  return (double)78.125 * (double)i2c_readInt24(INA_REGISTER::VSHUNT) / 1e+6;
}

//Energy in JOULES
double Energy()
{
  uint64_t energy = i2c_readUint40(INA_REGISTER::ENERGY);
  return 16.0 * 3.2 * CURRENT_LSB * energy;
}

// Charge in Coulombs
double Charge()
{
  int64_t charge = i2c_readInt40(INA_REGISTER::CHARGE);
  return CURRENT_LSB * (int32_t)charge;
}

double Power()
{
  //Calculated power output.
  //Output value in watts.
  //Unsigned representation. Positive value.
  //POWER Power [W] = 3.2 x CURRENT_LSB x POWER
  return (double)i2c_readUint24(INA_REGISTER::POWER) * (double)3.2 * CURRENT_LSB;
}

double DieTemperature()
{
  //The INA228 device has an internal temperature sensor which can measure die temperature from –40 °C to +125
  //°C. The accuracy of the temperature sensor is ±2 °C across the operational temperature range. The temperature
  //value is stored inside the DIETEMP register and can be read through the digital interface
  //Internal die temperature measurement. Two's complement value. Conversion factor: 7.8125 m°C/LSB

  //Case unsigned to int16 to cope with negative temperatures
  int16_t dietemp = i2c_readword(INA_REGISTER::DIETEMP);

  return dietemp * (double)7.8125 / (double)1000.0;
}

double Current()
{
  //Current.
  //Calculated current output in Amperes. Two's complement value.
  return CURRENT_LSB * (double)i2c_readInt24(INA_REGISTER::CURRENT);
}

ISR(PORTB_PORT_vect)
{
  uint8_t flags = PORTB.INTFLAGS;
  PORTB.INTFLAGS = flags; //clear flags

  if (flags && PIN1_bm)
  {
    //INA228 has triggered an ALERT interrupt
    RedLED(true);
    ALERT_TRIGGERED = true;
  }
}

void SendModbusData(uint8_t *sendbuff, const uint8_t len)
{
  // calc checksum
  uint16_t csum = ModbusRTU_CRC(sendbuff, len);

  // insert checksum
  sendbuff[len] = csum & 0xFF;
  sendbuff[len + 1] = csum >> 8;

  // send over rs485 wire
  Serial.write(sendbuff, 2 + len);
}

void extractHighWord(double *value, const uint8_t index)
{
  //Convert double to an array of bytes
  uint8_t *i = (uint8_t *)value;
  sendbuff[index] = i[0 + 1];
  sendbuff[index + 1] = i[0 + 0];
}

void extractLowWord(double *value, const uint8_t index)
{
  //Convert double to an array of bytes
  uint8_t *i = (uint8_t *)value;
  sendbuff[index] = i[2 + 1];
  sendbuff[index + 1] = i[2 + 0];
}

//Modbus command 2 Read Discrete Inputs
uint8_t ReadDiscreteInputs(uint16_t address, uint16_t quantity)
{
  uint8_t ptr = 3;

  //Safety check, only return max 32 registers
  if (quantity > 32)
  {
    quantity = 32;
  }

  uint16_t config = i2c_readword(INA_REGISTER::CONFIG);
  uint16_t adc_config = i2c_readword(INA_REGISTER::ADC_CONFIG);

  uint8_t bitPosition = 0;
  for (size_t i = address; i < address + quantity; i++)
  {
    bool outcome = false;
    switch (i)
    {

    case 0:
    {
      outcome = diag_alrt_value & bit(DIAG_ALRT_FIELD::TMPOL);
      break;
    }
    case 1:
    {
      outcome = diag_alrt_value & bit(DIAG_ALRT_FIELD::SHNTOL);
      break;
    }
    case 2:
    {
      outcome = diag_alrt_value & bit(DIAG_ALRT_FIELD::SHNTUL);
      break;
    }
    case 3:
    {
      outcome = diag_alrt_value & bit(DIAG_ALRT_FIELD::BUSOL);
      break;
    }
    case 4:
    {
      outcome = diag_alrt_value & bit(DIAG_ALRT_FIELD::BUSUL);
      break;
    }
    case 5:
    {
      outcome = diag_alrt_value & bit(DIAG_ALRT_FIELD::POL);
      break;
    }

    case 6:
    {
      //Temperature compensation
      outcome = config & bit(5);
      break;
    }

    case 7:
    {
      //ADC Range
      //0h = ±163.84 mV, 1h = ± 40.96 mV
      outcome = config & bit(4);
      break;
    }

      //Output bits for relay_trigger_bitmap
    case 8:
    {
      outcome = registers.relay_trigger_bitmap & bit(DIAG_ALRT_FIELD::TMPOL);
      break;
    }
    case 9:
    {
      outcome = registers.relay_trigger_bitmap & bit(DIAG_ALRT_FIELD::SHNTOL);
      break;
    }
    case 10:
    {
      outcome = registers.relay_trigger_bitmap & bit(DIAG_ALRT_FIELD::SHNTUL);
      break;
    }
    case 11:
    {
      outcome = registers.relay_trigger_bitmap & bit(DIAG_ALRT_FIELD::BUSOL);
      break;
    }
    case 12:
    {
      outcome = registers.relay_trigger_bitmap & bit(DIAG_ALRT_FIELD::BUSUL);
      break;
    }
    case 13:
    {
      outcome = registers.relay_trigger_bitmap & bit(DIAG_ALRT_FIELD::POL);
      break;
    }

    case 14:
    {
      outcome = relay_state;
      break;
    }

    default:
    {
      break;
    }
    }

    if (outcome)
    {
      sendbuff[ptr] = sendbuff[ptr] | (1 << bitPosition);
    }

    bitPosition++;
    if (bitPosition == 8)
    {
      bitPosition = 0;
      ptr++;
    }
  }

  return ptr;
}

uint8_t ReadHoldingRegisters(uint16_t address, uint16_t quantity)
{
  //Populate data from byte 3...
  uint8_t ptr = 3;

  //Temporary variables to hold the register data
  double v = 0;
  double c = 0;
  double p = 0;
  double shuntv = 0;

  double BusOverVolt = 0;
  double BusUnderVolt = 0;
  double ShuntOverCurrentLimit = 0;
  double ShuntUnderCurrentLimit = 0;
  double PowerLimit = 0;

  //Safety check, only return max 32 registers
  if (quantity > 48)
  {
    quantity = 48;
  }

  for (size_t i = address; i < address + quantity; i++)
  {
    switch (i)
    {
    case 0:
    {
      //Voltage
      v = BusVoltage();
      extractHighWord(&v, ptr);
      break;
    }

    case 1:
    {
      //Voltage
      extractLowWord(&v, ptr);
      break;
    }

    case 2:
    {
      //Current
      c = Current();
      extractHighWord(&c, ptr);
      break;
    }

    case 3:
    {
      //Current
      extractLowWord(&c, ptr);
      break;
    }

    case 4:
    {
      //amphour_out
      extractHighWord(&amphour_out, ptr);
      break;
    }

    case 5:
    {
      //amphour_out
      extractLowWord(&amphour_out, ptr);
      break;
    }

    case 6:
    {
      //amphour_in
      extractHighWord(&amphour_in, ptr);
      break;
    }

    case 7:
    {
      //amphour_in
      extractLowWord(&amphour_in, ptr);
      break;
    }

    case 8:
    {
      //temperature
      int16_t t = (int16_t)DieTemperature();
      sendbuff[ptr] = (uint8_t)(t >> 8);
      sendbuff[ptr + 1] = (uint8_t)(t & 0x00FF);
      break;
    }

    case 9:
    {
      //Spare!
      break;
    }

    case 10:
    {
      //Power
      p = Power();
      extractHighWord(&p, ptr);
      break;
    }

    case 11:
    {
      //Power
      extractLowWord(&p, ptr);
      break;
    }

    case 12:
    {
      //Shunt mV
      shuntv = ShuntVoltage();
      extractHighWord(&shuntv, ptr);
      break;
    }

    case 13:
    {
      //Shunt mV
      extractLowWord(&shuntv, ptr);
      break;
    }

    case 14:
    {
      extractHighWord(&CURRENT_LSB, ptr);
      break;
    }

    case 15:
    {
      extractLowWord(&CURRENT_LSB, ptr);
      break;
    }
    case 16:
    {
      extractHighWord(&RSHUNT, ptr);
      break;
    }

    case 17:
    {
      extractLowWord(&RSHUNT, ptr);
      break;
    }

    case 18:
    {
      sendbuff[ptr] = (uint8_t)(shunt_max_current >> 8);
      sendbuff[ptr + 1] = (uint8_t)(shunt_max_current & 0x00FF);
      break;
    }
    case 19:
    {
      sendbuff[ptr] = (uint8_t)(shunt_millivolt >> 8);
      sendbuff[ptr + 1] = (uint8_t)(shunt_millivolt & 0x00FF);
      break;
    }

    case 20:
    {
      //SHUNT_CAL register
      uint16_t shunt_cal_value = i2c_readword(INA_REGISTER::SHUNT_CAL);
      sendbuff[ptr] = (uint8_t)(shunt_cal_value >> 8);
      sendbuff[ptr + 1] = (uint8_t)(shunt_cal_value & 0x00FF);
      break;
    }

    case 21:
    {
      //temperature limit
      int16_t t = i2c_readword(INA_REGISTER::TEMP_LIMIT);
      sendbuff[ptr] = (uint8_t)(t >> 8);
      sendbuff[ptr + 1] = (uint8_t)(t & 0x00FF);
      break;
    }

    case 22:
    {
      //Sets the threshold for comparison of the value to detect Bus Overvoltage (overvoltage protection).
      //Unsigned representation, positive value only. Conversion factor: 3.125 mV/LSB.
      BusOverVolt = (double)i2c_readword(INA_REGISTER::BOVL) * 0.003125F;
      extractHighWord(&BusOverVolt, ptr);
      break;
    }

    case 23:
    {
      extractLowWord(&BusOverVolt, ptr);
      break;
    }

    case 24:
    {
      BusUnderVolt = (double)i2c_readword(INA_REGISTER::BUVL) * 0.003125F;
      extractHighWord(&BusUnderVolt, ptr);
      break;
    }

    case 25:
    {
      extractLowWord(&BusUnderVolt, ptr);
      break;
    }

    case 26:
    {
      //Shunt Over Voltage Limit (current limit)
      int16_t value = i2c_readword(INA_REGISTER::SOVL);

      //const double x = (0.725 / full_scale_current) * full_scale_adc;
      //int16_t CurrentOverThreshold = (x * 1000.0 / 1.24);

      //1.25 µV/LSB
      ShuntOverCurrentLimit = ((double)value / 1000 * 1.25) / full_scale_adc * full_scale_current;

      extractHighWord(&ShuntOverCurrentLimit, ptr);
      break;
    }
    case 27:
    {
      extractLowWord(&ShuntOverCurrentLimit, ptr);
      break;
    }

    case 28:
    {
      //Shunt Over Voltage Limit (current limit)
      int16_t value = i2c_readword(INA_REGISTER::SUVL);

      //const double x = (0.725 / full_scale_current) * full_scale_adc;
      //int16_t CurrentOverThreshold = (x * 1000.0 / 1.24);

      ShuntUnderCurrentLimit = ((double)value / 1000 * 1.25) / full_scale_adc * full_scale_current;

      extractHighWord(&ShuntUnderCurrentLimit, ptr);
      break;
    }
    case 29:
    {
      extractLowWord(&ShuntUnderCurrentLimit, ptr);
      break;
    }

    case 30:
    {
      //Shunt Over Voltage Limit (current limit)
      uint16_t value = i2c_readword(INA_REGISTER::PWR_LIMIT);
      PowerLimit = value * 256;
      extractHighWord(&PowerLimit, ptr);
      break;
    }
    case 31:
    {
      extractLowWord(&PowerLimit, ptr);
      break;
    }

      //These settings would probably be better in a 0x2B function code
      //https://modbus.org/docs/Modbus_Application_Protocol_V1_1b.pdf
    case 49:
    {
      //GITHUB version
      sendbuff[ptr] = (uint8_t)(GIT_VERSION_B1 >> 8);
      sendbuff[ptr + 1] = (uint8_t)(GIT_VERSION_B1 & 0x00FF);
      break;
    }
    case 50:
    {
      //GITHUB version
      sendbuff[ptr] = (uint8_t)(GIT_VERSION_B2 >> 8);
      sendbuff[ptr + 1] = (uint8_t)(GIT_VERSION_B2 & 0x00FF);
      break;
    }

    case 51:
    {
      //COMPILE_DATE_TIME_EPOCH
      sendbuff[ptr] = (uint8_t)(COMPILE_DATE_TIME_UTC_EPOCH >> 8);
      sendbuff[ptr + 1] = (uint8_t)(COMPILE_DATE_TIME_UTC_EPOCH & 0x00FF);
      break;
    }
    case 52:
    {
      //COMPILE_DATE_TIME_EPOCH
      sendbuff[ptr] = (uint8_t)(COMPILE_DATE_TIME_UTC_EPOCH >> 24);
      sendbuff[ptr + 1] = (uint8_t)(COMPILE_DATE_TIME_UTC_EPOCH >> 16 & 0x00FF);
      break;
    }

    case 53:
    {
      //INAXXX chip model number (should always be 0x0228)
      uint16_t dieid = i2c_readword(INA_REGISTER::DIE_ID);
      dieid = (dieid & 0xFFF0) >> 4;
      sendbuff[ptr] = (uint8_t)(dieid >> 8);
      sendbuff[ptr + 1] = (uint8_t)(dieid & 0x00FF);
      break;
    }

    default:
    {
      break;
    }
    } //end switch

    ptr += 2;
  } //end for

  return ptr;
}

unsigned long timer = 0;

#define combineBytes(high, low) (high << 8) + low

void loop()
{
  wdt_reset();

  /*
  //Enable interrupts
  sei();
  //Start frame detection
  USART0.CTRLB |= USART_SFDEN_bm;

  //Switch off TX, saves current
  //DisableSerial0TX();

  if (!Serial.available())
  {
    //Enter sleep
    set_sleep_mode(SLEEP_MODE_STANDBY);
    sleep_enable();
    sleep_cpu();

    //Snoring can be heard at this point....
    sleep_disable();
  }
*/

  if (ALERT_TRIGGERED)
  {
    ALERT_TRIGGERED = false;
    diag_alrt_value = i2c_readword(INA_REGISTER::DIAG_ALRT);

    alert = diag_alrt_value & (bit(DIAG_ALRT_FIELD::TMPOL) |
                               bit(DIAG_ALRT_FIELD::SHNTOL) |
                               bit(DIAG_ALRT_FIELD::SHNTUL) |
                               bit(DIAG_ALRT_FIELD::BUSOL) |
                               bit(DIAG_ALRT_FIELD::BUSUL) |
                               bit(DIAG_ALRT_FIELD::POL));

    if (alert == 0)
    {
      RedLED(false);
    }

    //Apply relay_trigger_bitmap bitmask over the top of the alerts, so we only trigger on specific events
    relay_state = ((alert & registers.relay_trigger_bitmap) != 0);

    //Turn relay on/off
    if (relay_state)
    {
      PORTA.OUTSET = RELAY_PIN_BITMAP;
    }
    else
    {
      PORTA.OUTCLR = RELAY_PIN_BITMAP;
    }
  }

  while (Serial.available())
  {
    GreenLED(true);

    // we have a rolling 8 byte window
    for (uint8_t k = 1; k < 8; k++)
    {
      modbus[k - 1] = modbus[k];
    }

    modbus[7] = (uint8_t)Serial.read(); // receive a byte

    //3 = Read Holding Registers
    if (
        (modbus[0] == ModbusSlaveAddress) &&
        ((modbus[1] == 2) || (modbus[1] == 3)))
    {
      //Do something

      uint16_t crc16 = combineBytes(modbus[7], modbus[6]);

      uint16_t calculatedCRC = ModbusRTU_CRC(modbus, 6);
      if (crc16 == calculatedCRC)
      {
        //EnableSerial0TX();

        //Prepare reply buffer
        memset(sendbuff, 0, sizeof(sendbuff));
        sendbuff[0] = ModbusSlaveAddress; // slv addr
        sendbuff[1] = modbus[1];
        uint8_t ptr = 0;

        //1 = command
        //2+3 = data address
        //4+5 = data amount/quantity
        if (modbus[1] == 2)
        {
          ptr = ReadDiscreteInputs(combineBytes(modbus[2], modbus[3]), combineBytes(modbus[4], modbus[5]));
        }

        if (modbus[1] == 3)
        {
          ptr = ReadHoldingRegisters(combineBytes(modbus[2], modbus[3]), combineBytes(modbus[4], modbus[5]));
        }

        sendbuff[2] = ptr - 3;
        SendModbusData(&sendbuff[0], ptr);
      }
      GreenLED(false);
    }
  }

  if (millis() > timer)
  {

    RedLED(true);

    //Check again in 2 seconds
    timer = millis() + 5000;

    //double voltage = BusVoltage();
    //150A@50mV shunt =   122.88A @ 40.96mV (full scale ADC)
    //double current = Current();
    double power = Power();
    //double temperature = DieTemperature();
    //double shuntv = ShuntVoltage();
    //double energy_joules = Energy();
    double charge_coulombs = Charge();

    //8202
    if (power > 0)
    {
      if (charge_coulombs > 0)
      {
        amphour_out += charge_coulombs * CoulombsToAmpHours;
      }
      else
      {
        amphour_in += charge_coulombs * CoulombsToAmpHours;
      }
    }

    ResetChargeEnergyRegisters();

    if (alert == 0)
    {
      //Turn LED off if alert is not active
      RedLED(false);
    }
  }
}