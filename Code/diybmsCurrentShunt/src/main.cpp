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

const uint8_t INA228_I2C_Address{B1000000};
const uint8_t INA_REGISTER_CONFIG{0}; ///< Configuration Register address
const uint8_t INA_REGISTER_MANUFACTURER_ID{0xFE};
const uint8_t INA_REGISTER_DIE_ID{0xFF};
const uint16_t INA_RESET_DEVICE{0x8000};

void RedLED(bool value);
void GreenLED(bool value);

volatile bool wdt_triggered = false;
volatile uint16_t wdt_triggered_count;

/*
ISR(PORTB_PORT_vect)
{
  if (PORTB.INTFLAGS && PIN1_bm)
  {
    //INA228 has triggered an ALERT interrupt

    RedLED(true);
  }

  //Reset interrupt
  PORTB.INTFLAGS = 0xFF;
}
*/

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
  //PORTB.PIN1CTRL=PORT_PULLUPEN_bm | PORT_ISC_enum::PORT_ISC_FALLING_gc;
  //PORTB.PIN1CTRL = PORT_ISC_enum::PORT_ISC_FALLING_gc;
  //SREG = 0b10000000;
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
  Wire.setClock(400000);

  Wire.beginTransmission(INA228_I2C_Address); // transmit to device #8
  uint8_t result = Wire.endTransmission();

  if (result > 0)
  {
    i2c_error();
  }

  //Now we know the INA228 is alive and connected, set it up
  if (!i2c_writeword(INA_REGISTER_CONFIG, INA_RESET_DEVICE))
  {
    i2c_error();
  }

  //Get the INA chip model number (make sure we are dealing with an INA228)

  //Clear lower 3 bits, holds version/chip revision
  int16_t dieid = i2c_readword(INA_REGISTER_DIE_ID);

#ifdef DIYBMS_DEBUG
  debugSerial.println(dieid, HEX);
#endif

  dieid = (dieid & 0xFFF0) >> 4;

#ifdef DIYBMS_DEBUG
  debugSerial.println(dieid, HEX);
#endif

  //INA228 chip
  if (dieid != 0x228)
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

#ifdef DIYBMS_DEBUG
uint16_t loopCount = 0;
#endif

void loop()
{
  wdt_reset();

  GreenLED(false);
  delay(250);
  GreenLED(true);
  delay(250);

#ifdef DIYBMS_DEBUG
  debugSerial.print('#');

  if (loopCount % 48 == 0)
  {
    debugSerial.println();
  }

  loopCount++;
#endif
}