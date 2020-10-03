#include <Arduino.h>

void setup() {
  // put your setup code here, to run once:
  pinMode(PD6,OUTPUT);
  pinMode(PD7,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(PD6,LOW);
  digitalWrite(PD7,HIGH);
  delay(50);
  digitalWrite(PD6,HIGH);
  digitalWrite(PD7,LOW);
  delay(50);
}