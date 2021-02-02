#include <Arduino.h>

#include <MCP2515.h>

MCP2515 mcp2515(10);

MCP2515::ERROR err;

struct can_frame canMsg1;
struct can_frame canMsg2;

void setup()
{
  SPI.begin();

  canMsg1.can_id = 0x0F6;
  canMsg1.can_dlc = 8;
  canMsg1.data[0] = 1;
  canMsg1.data[1] = 2;
  canMsg1.data[2] = 3;
  canMsg1.data[3] = 4;
  canMsg1.data[4] = 5;
  canMsg1.data[5] = 6;
  canMsg1.data[6] = 7;
  canMsg1.data[7] = 8;
  /*
  canMsg2.can_id = 0x036;
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0x0E;
  canMsg2.data[1] = 0x00;
  canMsg2.data[2] = 0x00;
  canMsg2.data[3] = 0x08;
  canMsg2.data[4] = 0x01;
  canMsg2.data[5] = 0x00;
  canMsg2.data[6] = 0x00;
  canMsg2.data[7] = 0xA0;
*/
  //pinMode(13, OUTPUT);

  pinMode(PB0, OUTPUT);
  pinMode(5, OUTPUT);
  Serial.begin(115200, SERIAL_8N1);

  err = mcp2515.reset();
  Serial.println(err);
  err = mcp2515.setBitrate(CAN_125KBPS, MCP_16MHZ);
  Serial.println(err);
  err = mcp2515.setNormalMode();
  //err = mcp2515.setLoopbackMode();
  Serial.println(err);

  Serial.println(F("Start up"));
}

void dumpByte(uint8_t data)
{
  if (data <= 0x0F)
  {
    Serial.print('0');
  }
  Serial.print(data, HEX);
}

void loop()
{
  canMsg1.data[0]++;
  err = mcp2515.sendMessage(&canMsg1);
  Serial.print(err);
  Serial.print(' ');
  //err = mcp2515.sendMessage(&canMsg2);
  //Serial.print(err);
  //Serial.print(',');

  if (err == 0)
  {
    digitalWrite(5, HIGH);
    delay(100);
    digitalWrite(5, LOW);
  }
  else
  {
    Serial.print(" reset ");
  
    err = mcp2515.reset();
    Serial.print(err);
    err = mcp2515.setBitrate(CAN_125KBPS, MCP_16MHZ);
    Serial.print(err);
    err = mcp2515.setNormalMode();
    Serial.println(err);
  
  }
/*
  if (mcp2515.checkReceive())
  {
    err = mcp2515.readMessage(&canMsg2);

    if (err == 0)
    {

      Serial.printf("ID is %d=", canMsg2.can_id);
      //if (!(canMsg2.flags & CAN_MSG_FLAG_RTR))
      //{
      for (int i = 0; i < canMsg2.can_dlc; i++)
      {
        dumpByte(canMsg2.data[i]);
      }
      //}
      Serial.println();
    }
  }
  */

  delay(5000);
}