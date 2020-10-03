#include <Arduino.h>
#include<avr/wdt.h> /* Header for watchdog timers in AVR */
#include <Wire.h>
#include "LTC2944.h"

const unsigned int fullCapacity = 240; // Maximum value is 22000 mAh

LTC2944 gauge(50); // Takes R_SENSE value (in milliohms) as constructor argument, can be omitted if using CJMCU-294

void setup() {
  wdt_disable();
  wdt_reset();
  wdt_enable(WDTO_4S);

  // put your setup code here, to run once:
  pinMode(PD6,OUTPUT);
  pinMode(PD7,OUTPUT);

  //Flash both LEDS on power up
  for (size_t i = 0; i < 20; i++)
  {
    digitalWrite(PD6,HIGH);
    digitalWrite(PD7,HIGH);
    delay(50);
    digitalWrite(PD6,LOW);
    digitalWrite(PD7,LOW);
    delay(50);
  }
  

  Wire.begin();

  while (gauge.begin() == false) {
    wdt_reset();
    //Flash blue LED slowly - bad news
    digitalWrite(PD7,HIGH);
    delay(500);
    digitalWrite(PD7,LOW);
    delay(500);
  }

  gauge.setBatteryCapacity(fullCapacity);
  gauge.setBatteryToFull(); // Sets accumulated charge registers to the maximum value
  gauge.setADCMode(ADC_MODE_SLEEP); // In sleep mode, voltage and temperature measurements will only take place when requested
  gauge.startMeasurement();
}

void loop() {
  wdt_reset();

  // put your main code here, to run repeatedly:
  digitalWrite(PD6,HIGH);
  unsigned int raw = gauge.getRawAccumulatedCharge();
  Serial.print(F("Raw Accumulated Charge: "));
  Serial.println(raw, DEC);

  float capacity = gauge.getRemainingCapacity();
  Serial.print(F("Battery Capacity: "));
  Serial.print(capacity, 3);
  Serial.print(F(" / "));
  Serial.print(fullCapacity, DEC);
  Serial.println(F(" mAh"));

  float voltage = gauge.getVoltage();
  Serial.print(F("Voltage: "));
  Serial.print(voltage, 3);
  Serial.println(F(" V"));

  float temperature = gauge.getTemperature();
  Serial.print(F("Temperature: "));
  Serial.print(temperature, 2);
  Serial.println(F(" 'C"));

  Serial.println();

  digitalWrite(PD6,LOW);
  delay(2000);  
}