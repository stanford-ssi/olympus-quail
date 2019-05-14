#include <Arduino.h>
#include <pin_interface.h>
#include "MC33797.h"
#include <SPI.h>
#include "wiring_private.h"

const int slaveSelectPin = 10;

SPIClass squibSPI (&sercom0, Squib_MISO, Squib_SCK, Squib_MOSI,
                   SPI_PAD_0_SCK_1, SERCOM_RX_PAD_2);

void setup()
{
  Serial.begin(9600);
  pinPeripheral(Squib_MISO, PIO_SERCOM_ALT);
  pinPeripheral(Squib_SCK, PIO_SERCOM_ALT);
  pinPeripheral(Squib_MOSI, PIO_SERCOM_ALT);

  pinMode(Squib_SS_1, OUTPUT);
  pinMode(Squib_SS_2, OUTPUT);
  digitalWrite(Squib_SS_1, HIGH);
  digitalWrite(Squib_SS_2, HIGH);
  squibSPI.begin();

  delay(3000);

  // Doesn't work for some reason: seems to only return back the command that was sent to it
  // > send 0x96, receive 0xC8
  // > send 0xC8, receive 0x96
  // > send 0x96, receive 0xC8
  // > etc...

  // only returns 0x96 over and over (because that is the command sent)
  // while (true) {
    // delay(100);
    // uint8_t send = 0x96;
    // uint8_t recv = 0x00;
    // squibSPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    // digitalWrite(Squib_SS_2, LOW);
    // recv = squibSPI.transfer(send);
    // squibSPI.endTransaction();
    // digitalWrite(Squib_SS_2, HIGH);
// 
    // Serial.print("Squib Response: 0x");
    // Serial.println(recv, HEX);
    // Serial.println(recv == 0x69);
    // Serial.println();
  // }

  


  
  uint8_t ret = Squib_Init(2);
  // while (ret > 0) {
    // delay(100);
    // ret = Squib_Init(2);
    // Serial.println();
  // }
  Serial.print("Squib 2 Init: ");
  Serial.println(ret);

  delay(2000);

  ret = Squib_Init(1);
  while (ret > 0) {
    delay(100);
    ret = Squib_Init(1);
    Serial.println();
  }
  Serial.print("Squib 1 Init: ");
  Serial.println(ret);

  Serial.println();
}

void loop()
{
  delay(1000);

  Squib_StatusType *s = new Squib_StatusType();

  uint8_t ret;

  ret = Squib_GetStatus(s, 1);
  Serial.print("Status: ");
  Serial.println(ret);
  Serial.print("1A Resistance: ");
  Serial.println(s->Squib_Stat1AResistance, BIN);
  Serial.print("1B Resistance: ");
  Serial.println(s->Squib_Stat1BResistance, BIN);
  Serial.print("2A Resistance: ");
  Serial.println(s->Squib_Stat2AResistance, BIN);
  Serial.print("2B Resistance: ");
  Serial.println(s->Squib_Stat2BResistance, BIN);

  Serial.print("Enable 1: ");
  Serial.println(s->Squib_StatFen1);
  Serial.print("Enable 2: ");
  Serial.println(s->Squib_StatFen2);

  /*
  if (Serial.available())
  {
    int code = Serial.parseInt();
    if (code == 1)
      ret = Squib_Fire(CMD_FIRE_1A);
    if (code == 2)
      ret = Squib_Fire(CMD_FIRE_1B);
    if (code == 3)
      ret = Squib_Fire(CMD_FIRE_2A);
    if (code == 4)
      ret = Squib_Fire(CMD_FIRE_2B);
    // Serial.print("Fire: ");
    Serial.println(ret);
  }
  */
}

extern "C"
{
  void debug(const char *data)
  {
    Serial.println(data);
  }

  void debug_hex(uint8_t data)
  {
    Serial.println(data, HEX);
  }

  void debug_bin(uint8_t data)
  {
    Serial.println(data, BIN);
  }

  uint8_t send(uint8_t data)
  {
    // digitalWrite(whichSquib, LOW);
    squibSPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    uint8_t val = squibSPI.transfer(data);
    squibSPI.endTransaction();
    // digitalWrite(whichSquib, HIGH);
    return val;
  }
}
