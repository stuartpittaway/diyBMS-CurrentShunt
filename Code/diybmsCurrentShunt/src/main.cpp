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
*/

// ATTINY1614 (tinyAVR® 1-series of microcontrollers)
// http://ww1.microchip.com/downloads/en/DeviceDoc/ATtiny1614-data-sheet-40001995A.pdf

//MODBUS Protocol
//https://www.ni.com/en-gb/innovations/white-papers/14/the-modbus-protocol-in-depth.html

#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <Wire.h>

#include "EmbeddedFiles_Defines.h"

#define DIYBMS_DEBUG 1

#ifdef DIYBMS_DEBUG
//Just for debug purposes
#include "SendOnlySoftwareSerial.h"
SendOnlySoftwareSerial debugSerial(0); // Tx pin
#endif

#define GREENLED_PIN_BITMAP PIN7_bm
#define REDLED_PIN_BITMAP PIN6_bm
#define RELAY_PIN_BITMAP PIN5_bm

//150A@50mV shunt =   122.88A @ 40.96mV (full scale ADC)
const double shunt_max_current = 150.0;
const double shunt_millivolt = 50.0;
const double full_scale_current = shunt_max_current / shunt_millivolt * 40.96;
const double CURRENT_LSB = full_scale_current / (double)0x80000;
const double RSHUNT = (shunt_millivolt / 1000.0) / shunt_max_current;

const uint8_t INA228_I2C_Address{B1000000};

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

//const uint16_t INA_RESET_DEVICE{0x8000};
const uint16_t INA_RESET_DEVICE{_BV(15)};

void RedLED(bool value);
void GreenLED(bool value);

volatile bool wdt_triggered = false;
volatile uint16_t wdt_triggered_count;

volatile bool i = false;
ISR(PORTB_PORT_vect)
{
  if (PORTB.INTFLAGS && PIN1_bm)
  {
    //INA228 has triggered an ALERT interrupt
    i = !i;
    RedLED(i);
  }

  //Reset interrupt
  PORTB.INTFLAGS = 0xFF;
}

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

  // Set Port B digital outputs
  // PB0 = XDIR (RS485 transmit enable)
  // PB1 = ALERT
  // PB2 = TX (has to be set as output on tiny1614)
  // PB3 = RX
  // Set pins as Outputs (other pins are inputs)
  PORTB.DIR = PIN0_bm | PIN2_bm;
  PORTB.PIN1CTRL = PORT_PULLUPEN_bm | PORT_ISC_enum::PORT_ISC_FALLING_gc;
  //PORTB.PIN1CTRL = PORT_ISC_enum::PORT_ISC_FALLING_gc;

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

  // Setup a watchdog timer for 2 seconds
  CCP = 0xD8;

  //2 seconds
  WDT.CTRLA = WDT_PERIOD_enum::WDT_PERIOD_4KCLK_gc;
  //WDT.CTRLA = WDT_PERIOD_enum::WDT_PERIOD_128CLK_gc;

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
  return ((uint16_t)Wire.read() << 8) | Wire.read();
}

uint32_t i2c_readUint24(const uint8_t inareg)
{
  Wire.beginTransmission(INA228_I2C_Address);
  Wire.write(inareg);
  Wire.endTransmission();
  delayMicroseconds(10);
  Wire.requestFrom(INA228_I2C_Address, (uint8_t)3); // Request 3 bytes

  uint32_t reply = (uint32_t)Wire.read() << 16;
  reply += (uint32_t)Wire.read() << 8;
  reply += (uint32_t)Wire.read();

  return reply;
}

void dumpByte(uint8_t data)
{
  if (data <= 0x0F)
    debugSerial.print('0');
  debugSerial.print(data, HEX);
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

  dumpByte(a);
  dumpByte(b);
  dumpByte(c);
  dumpByte(d);
  dumpByte(e);

  uint64_t reply = (uint64_t)a << 32;
  reply += (uint64_t)b << 24;
  reply += (uint64_t)c << 16;
  reply += (uint64_t)d << 8;
  reply += (uint64_t)e;

  return reply;
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

#ifdef DIYBMS_DEBUG
  debugSerial.println("I2C Error");
#endif

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

  //Now we know the INA228 is connected, reset to defaults
  if (!i2c_writeword(INA_REGISTER::CONFIG, INA_RESET_DEVICE))
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
#ifdef DIYBMS_DEBUG
    debugSerial.println(dieid, HEX);
#endif
    i2c_error();
  }

  const uint16_t value = _BV(4); //Enable ADCRANGE = 40.96mV scale

  if (!i2c_writeword(INA_REGISTER::CONFIG, value))
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

  //shuntcal_value+=300;
  //Top 2 bits are not used
  shunt_cal_value = shunt_cal_value & 0x3FFF;
#ifdef DIYBMS_DEBUG
  debugSerial.println(shunt_cal_value);
#endif

  if (!i2c_writeword(INA_REGISTER::SHUNT_CAL, shunt_cal_value))
  {
    i2c_error();
  }

  //CNVR = Setting this bit high configures the Alert pin to be asserted when the Conversion Ready Flag (bit 1) is asserted, indicating that a conversion cycle has completed
  const uint16_t diag_alrt_value = 0; //_BV(14);
  if (!i2c_writeword(INA_REGISTER::DIAG_ALRT, diag_alrt_value))
  {
    i2c_error();
  }
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

  ConfigurePorts();

  //Serial uses PB2/PB3 and PB0 for XDIR
  Serial.begin(19200, SERIAL_8N1);
  //Serial.swap(0);

  //0x01= Enables RS-485 mode with control of an external line driver through a dedicated Transmit Enable (TE) pin.
  USART0.CTRLA |= B00000001;

  for (size_t i = 0; i < 25; i++)
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

  EnableWatchdog();

  ReadJumperPins();

#ifdef DIYBMS_DEBUG
  //Steal the ADDRESS pin for a Serial debug output
  PORTA.DIRSET = PIN4_bm;
  debugSerial.begin(2400);
  debugSerial.println();
  debugSerial.println("DEBUG");
#endif

  ConfigureI2C();
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

void loop()
{
  wdt_reset();

  GreenLED(false);
  delay(250);
  GreenLED(true);
  delay(250);

  double voltage = BusVoltage();
  //150A@50mV shunt =   122.88A @ 40.96mV (full scale ADC)
  double current = Current();
  double temperature = DieTemperature();
  double power = Power();
  double shuntv = ShuntVoltage();

#ifdef DIYBMS_DEBUG
  //debugSerial.print('#');

  debugSerial.print(voltage, 6);
  debugSerial.print("   ");
  debugSerial.print(current, 6);
  debugSerial.print("   ");
  debugSerial.print(power, 3);
  debugSerial.print("W   ");
  debugSerial.print(shuntv, 6);
  debugSerial.print("mV   ");
  debugSerial.println(temperature, 6);

  //Calculated energy output. Output value is in Joules.Unsigned representation. Positive value.
  debugSerial.print("Energy=");
  uint64_t energy = i2c_readUint40(INA_REGISTER::ENERGY);
  double energy_joules = 16.0 * 3.2 * CURRENT_LSB * (uint32_t)energy;
  debugSerial.print(" ");
  debugSerial.print(energy_joules, 6);
  debugSerial.print("J ");

  //Calculated charge output. Output value is in Coulombs. Two's complement value
  debugSerial.print("  Charge=");
  uint64_t charge = i2c_readUint40(INA_REGISTER::CHARGE);
  double charge_coulombs = CURRENT_LSB * (uint32_t)charge;
  debugSerial.print(" ");
  debugSerial.print(charge_coulombs, 6);
  debugSerial.print("C ");
  debugSerial.println();
#endif
}