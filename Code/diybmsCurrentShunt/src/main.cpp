#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include "EmbeddedFiles_Defines.h"

//MODBUS Protocol
//https://www.ni.com/en-gb/innovations/white-papers/14/the-modbus-protocol-in-depth.html
//#define RX_BUFFER_SIZE 64

#define GREENLED_PIN_BITMAP PIN7_bm
#define REDLED_PIN_BITMAP PIN6_bm
#define RELAY_PIN_BITMAP PIN5_bm

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
  // Outputs
  PORTA.DIRSET = GREENLED_PIN_BITMAP | REDLED_PIN_BITMAP | RELAY_PIN_BITMAP;

  // Set Port B digital outputs
  // PB2 = TX (has to be set as output on tiny1614)
  PORTB.DIRSET = PIN2_bm;

  // Set RX as input (PB3)
  PORTB.DIRCLR = PIN3_bm;

  //PORTA.DIRCLR = PIN3_bm | PIN4_bm | PIN7_bm;
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

  Serial.begin(115200, SERIAL_8N1);
}

void loop()
{
  wdt_reset();

  GreenLED(false);
  RedLED(false);
  delay(1000);
  GreenLED(true);
  RedLED(true);
  delay(1000);
}