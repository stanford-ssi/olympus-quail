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

Solenoids oxidizerTankVent, nitrogenFill, nitrousFill, nitrousAbort, nitrogenAbort, pyrovalveShutOff;
PressureSensor nitrousLine, nitrousHeatXger, nitrogenLine, oxidizerTank, combustionChamber;
File dataFile;
double data;

Solenoids SolenoidArray[] = {oxidizerTankVent, nitrogenFill, nitrousFill, nitrousAbort, nitrogenAbort, pyrovalveShutOff}; // J12, J13, J15, J16, J19, J20
PressureSensor TransducerArray[] = {nitrousLine, nitrousHeatXger, nitrogenLine, oxidizerTank, combustionChamber};

const uint8_t SquibA = 1;
const uint8_t SquibB = 2;
const int IGNITION_DELAY = 4000;
unsigned long startTime = millis();
unsigned long ignitionTime;

bool IGNITION = false;

int functionNumber = -1;
int deviceNumber = -1;

String inData;
String lastCommand = "00";
int command;

const char logFileName[] = "ignition.txt"; // Must be 8 chars or less
ofstream output;

SPIClass squibSPI(&sercom0, Squib_MISO, Squib_SCK, Squib_MOSI, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_2);

//Want to move this to sepreate File
int PrintTansducerValuesSerial(PressureSensor transducers[], unsigned long timeMillis);
// Should log to SD too with time stamps
int LogTansducerValuesSD(PressureSensor transducers[], unsigned long timeMillis);
//Creates Packets to send to Print and Log functions
String createDataPacket(PressureSensor transducers[], unsigned long timeMillis, unsigned long loadCell, String lastCommand);

void setup()
{
  // put your setup code here, to run once:
  //pinMode(Solenoid_1, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Setup Begin");
  pinMode(LED_D2, OUTPUT);
  pinMode(LED_D3, OUTPUT);

  TransducerArray[0].initializeSensor(Pressure_1, RANGE_1K, 3);  // Nitrous Line/Supply 3
  TransducerArray[1].initializeSensor(Pressure_2, RANGE_2K, 0);  // Combustion Chamber
  TransducerArray[2].initializeSensor(Pressure_3, RANGE_1K, 0);  // Nitrogen Line 6
  TransducerArray[3].initializeSensor(Pressure_4, RANGE_2K, -5); // Oxidizier Tank (On Motor) 0
  TransducerArray[4].initializeSensor(Pressure_5, RANGE_2K, -2); // Combustion Chamber or Manifold Pressure -9

  SolenoidArray[0].initializeSolenoid(Solenoid_1, SOLENOID_MEDIUM, SECONDS_2); // Oxidizer Tank Vent, On Rocket, Edelbrook (J12)
  SolenoidArray[1].initializeSolenoid(Solenoid_2, SOLENOID_SMALL, MILLIS_200); // Nitrogen Fill (J13)
  SolenoidArray[2].initializeSolenoid(Solenoid_3, SOLENOID_LARGE, SECONDS_2);  //  Nitrous Fill, Pro BigShot?? (J15)
  SolenoidArray[3].initializeSolenoid(Solenoid_4, SOLENOID_ABORT, SECONDS_10); //  Nitrous Abort (J16)
  SolenoidArray[4].initializeSolenoid(Solenoid_5, SOLENOID_ABORT, SECONDS_10); //  Fuel/Nitrogen Abort (J19)
  SolenoidArray[5].initializeSolenoid(Solenoid_6, SOLENOID_ABORT, SECONDS_10); // Pyrovalve Shut Off(J20)

  pinMode(Analog_Sensor_In, INPUT);

  pinPeripheral(Squib_MISO, PIO_SERCOM_ALT);
  pinPeripheral(Squib_SCK, PIO_SERCOM_ALT);
  pinPeripheral(Squib_MOSI, PIO_SERCOM_ALT);

  pinMode(Squib_SS_1, OUTPUT);
  pinMode(Squib_SS_2, OUTPUT);
  digitalWrite(Squib_SS_1, HIGH);
  digitalWrite(Squib_SS_2, HIGH);
  squibSPI.begin();

  //digitalWrite(LED_D2, HIGH);
  digitalWrite(LED_D3, HIGH);
  delay(1000);

  // Squib Init
  uint8_t ret = Squib_Init(SquibA);
  Serial.print("Squib 1 Init: ");
  Serial.println(ret);
  ret = Squib_Init(SquibB);
  Serial.print("Squib 2 Init: ");
  Serial.println(ret);
  Serial.println();

  // SD Card Init - While get stuck here if no SD
  Serial.println("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SD_Card_SS, OUTPUT);
  digitalWrite(LED_D2, HIGH);
  digitalWrite(LED_D3, LOW);
  output.open(logFileName);
  if(!output.is_open()){
    Serial.println("file not found");
  }
  // see if the card is present and can be initialized:
  // if (!SD.begin(SD_Card_SS)) {
  //   Serial.println("Card failed, or not present");
  //   // don't do anything more:
  //   while (1) ;
  // }

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

void loop()
{
  // put your main code here, to run repeatedly:

  //Squib_StatusType *s = new Squib_StatusType();

  uint8_t ret;
  unsigned long currTime = millis();
  String data;

  //Serial.println(currTime);

  // Sends Data packet every 5 ms
  if ((currTime - 1) > startTime)
  {
    unsigned long loadCell = analogRead(Analog_Sensor_In);
    data = createDataPacket(TransducerArray, currTime, loadCell, lastCommand);
    //PrintTansducerValuesSerial(TransducerArray, currTime);

    //dataFile.println(data);
    //make sure to uncomment this before testing
    if ((currTime - 5) > startTime)
    {
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
  uint8_t send(uint8_t data)
  {
    squibSPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
    uint8_t val = squibSPI.transfer(data);
    squibSPI.endTransaction();
    return val;
  }
}

String createDataPacket(PressureSensor transducers[], unsigned long timeMillis, unsigned long loadCell, String lastCommand)
{

  String out = "0, ";

  for (int i = 0; i < 5; i++)
  {
    data = transducers[i].readSensor();

    if (data > 2000 || data < -200)
      data = -1;

    out = out + data + ",";
  }
  out = out + loadCell + ',';
  out = out + timeMillis + ",";
  out = out + lastCommand;

  return out;
}

int PrintTansducerValuesSerial(PressureSensor transducers[], unsigned long timeMillis)
{
  //Serial.println("Sending Data:");

  for (int i = 0; i < 5; i++)
  {
    data = transducers[i].readSensor();

    if (data > 2000 || data < -200)
      data = -1;

    Serial.print(data);
    Serial.print(",");
  }

  Serial.print(",");
  Serial.println("0");

  return 1;
}

int LogTansducerValuesSD(PressureSensor transducers[], unsigned long timeMillis)
{
  dataFile.print(timeMillis);
  dataFile.print(",");

  for (int i = 0; i < 5; i++)
  {
    data = transducers[i].readSensor();

    if (data > 2000 || data < -200)
      data = -1;

    dataFile.print(data);
    dataFile.print(",");
  }

  Serial.print(",");
  Serial.println("0");
  return 1;
}
