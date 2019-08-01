#include <Arduino.h>
#undef min
#undef max
#include <pin_interface.h>
#include <solenoids.hpp>
#include <pressure_sensor.hpp>
#include "SD.h"
#include "MC33797.h"
#include <SPI.h>
#include "wiring_private.h"
#include <iostream.>
#include <fstream.>
//#include <datalogger.hpp>

File dataFile;
double data;

unsigned long startTime = millis();
unsigned long ignitionTime;

bool IGNITION = false;

int functionNumber = -1;
int deviceNumber = -1;

String inData;
String lastCommand = "00";
int command;

const string logFileName = "ignition.txt"; // Must be 8 chars or less
ofstream output;

//Creates Packets to send to Print and Log functions
String createDataPacket(unsigned long timeMillis, unsigned long loadCell, String lastCommand);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(1000);
  Serial.println("Setup Begin");
  pinMode(LED_D2, OUTPUT);
  pinMode(LED_D3, OUTPUT);
  pinMode(Analog_Sensor_In, INPUT);

  digitalWrite(LED_D3, HIGH);
  delay(1000);

  // SD Card Init - While get stuck here if no SD
  Serial.println("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SD_Card_SS, OUTPUT);
  digitalWrite(LED_D2, HIGH);
  digitalWrite(LED_D3, LOW);
  output.open(logFileName);
  if(!output.is_open()) {
    Serial.println("file not found");
  }
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_Card_SS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
  }

  digitalWrite(LED_D2, HIGH);
  digitalWrite(LED_D3, LOW);

  //Open up the file we're going to log to!
  // dataFile = SD.open(logFileName, ((O_RDWR|O_APPEND)));
  // if (!dataFile) {
  //   Serial.println("error opening datalog.txt");
  //   // Wait forever since we cant write data
  //    while (!dataFile) {
  //       Serial.println("error opening datalog.txt");
  //       delay(500);
  //       dataFile = SD.open(logFileName, (O_RDWR|O_APPEND));

  //    }
  // }
  // dataFile.println("Data logging for Quail has begun yeet");
  Serial.println("Setup done");
}

void loop() {
  // put your main code here, to run repeatedly:
  unsigned long currTime = millis();
  String data;

  // Sends Data packet every 5 ms
  if ((currTime - 1) > startTime) {
    unsigned long loadCell = analogRead(Analog_Sensor_In);
    data = createDataPacket(currTime, loadCell, lastCommand);

    //dataFile.println(data);
    //make sure to uncomment this before testing
    if ((currTime - 5) > startTime) {
      startTime = currTime;
      Serial.println(data + "");
      output << data + '\n';
    }
  }
} // end of loop

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
}

String createDataPacket(unsigned long timeMillis, unsigned long loadCell, String lastCommand) {
  String out = "0, ";

  for (int i = 0; i < 5; i++) {
    out += "0, ";
  }
  out = out + loadCell + ",";
  out = out + timeMillis + ",";
  out = out + lastCommand;

  return out;
}