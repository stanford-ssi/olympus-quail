#include <Arduino.h>
#include <pin_interface.h>
#include <solenoids.hpp>
#include <pressure_sensor.hpp>
#include  "SD.h"
#include "MC33797.h"
#include <SPI.h>
#include "wiring_private.h"

Solenoids oxidizerTankVent, nitrogenFill, nitrousFill, nitrousAbort, nitrogenAbort, pyrovalveShutOff;
PressureSensor nitrousLine, nitrousHeatXger, nitrogenLine, oxidizerTank, combustionChamber;
File dataFile;
double data;

Solenoids SolenoidArray[]          = {oxidizerTankVent, nitrogenFill, nitrousFill, nitrousAbort, nitrogenAbort, pyrovalveShutOff}; // J12, J13, J15, J16, J19, J20
PressureSensor TransducerArray[]   = {nitrousLine, nitrousHeatXger, nitrogenLine, oxidizerTank, combustionChamber}; 

const uint8_t SquibA = 1;
const uint8_t SquibB = 2;

SPIClass squibSPI (&sercom0, Squib_MISO, Squib_SCK, Squib_MOSI, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_2);

void setup() {
  // put your setup code here, to run once:
  //pinMode(Solenoid_1, OUTPUT);
  Serial.begin(9600);
  Serial.println("Setup Begin");
  pinMode(LED_D2, OUTPUT);
  pinMode(LED_D3, OUTPUT);

  TransducerArray[0].initializeSensor(Pressure_1);
  TransducerArray[1].initializeSensor(Pressure_2);
  TransducerArray[2].initializeSensor(Pressure_3);
  TransducerArray[3].initializeSensor(Pressure_4);
  TransducerArray[4].initializeSensor(Pressure_5);

  SolenoidArray[0].initializeSolenoid(Solenoid_1, MEDIUM);
  SolenoidArray[1].initializeSolenoid(Solenoid_2, MEDIUM);
  SolenoidArray[2].initializeSolenoid(Solenoid_3, LARGE);
  SolenoidArray[3].initializeSolenoid(Solenoid_4, SMALL);
  SolenoidArray[4].initializeSolenoid(Solenoid_5, SMALL);
  SolenoidArray[5].initializeSolenoid(Solenoid_6, SMALL);

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
  delay(2000);
 
 // Squib Init
  uint8_t ret = Squib_Init(SquibA);
  Serial.print("Squib 1 Init: ");
  Serial.println(ret);
  ret = Squib_Init(SquibB);
  Serial.print("Squib 2 Init: ");
  Serial.println(ret);
  Serial.println();



 // SD Card Init - While get stuck here if no SD
  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SD_Card_SS, OUTPUT);
  digitalWrite(LED_D2,HIGH);
  digitalWrite(LED_D3,LOW);
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_Card_SS, SD_Card_MOSI, SD_Card_MISO, SD_Card_SCK)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
  }
  Serial.println("card initialized.");
  
  digitalWrite(LED_D2,HIGH);
  digitalWrite(LED_D3,LOW);

  // Open up the file we're going to log to!
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (!dataFile) {
    Serial.println("error opening datalog.txt");
    // Wait forever since we cant write data
     while (!dataFile) {
        Serial.println("error opening datalog.txt");
        delay(500);
        dataFile = SD.open("datalog-5-10-19.txt", FILE_WRITE);

     }
  }
  dataFile.println("Data logging for Quail has begun yeet");
  Serial.println("Setup done");
}

void loop() {
  // put your main code here, to run repeatedly:
 
  int functionNumber  =  -1;
  int deviceNumber =-1;

  Squib_StatusType *s = new Squib_StatusType();

  uint8_t ret;


  if (Serial.available() > 0)
  {
    functionNumber = Serial.read() - '0';
     /*
      subtract 1 from the recipe number to get the index in the array
    */ 
    Serial.println("");
   
    if(functionNumber == 1)
    {
      // Function 1 OPENS Solenoid and prints status
      Serial.println("Which Solenoid would you like to Open?:");
      while(!Serial.available()>0);
      deviceNumber = Serial.read() - '0';
      
      if(deviceNumber>=1 && deviceNumber<7 )
      {
        if(SolenoidArray[deviceNumber-1].openSolenoid())
        {     
        Serial.print("Solenoid "+(String)deviceNumber+ " Status: ");
        Serial.println(SolenoidArray[deviceNumber-1].getSolenoidStatus());
        }
        else
        {
          Serial.println("Failed to Open Solenoid??");
        }
        
      }
      else
      {
        Serial.println("Invalid Device number");
      }

      }
    else if(functionNumber == 2)
    {
      // Function 2 CLOSES Solenoid and prints status
      Serial.println("Which Solenoid would you like to Close?:");
      while(!Serial.available()>0);
      deviceNumber = Serial.read() - '0';

      if(deviceNumber>=1 && deviceNumber<7 )
      {
        SolenoidArray[deviceNumber-1].closeSolenoid();
        Serial.print("Solenoid "+(String)deviceNumber+ " Status: ");
        Serial.println(nitrousFill.getSolenoidStatus());
      }
      else
      {
        Serial.println("Invalid Device number");
      }
      
    }
    else if(functionNumber == 3)
    {
      // Function 3 READS & LOGS Pressure Transducer
      Serial.println("Which Transducer would you like to Read?:");
      while(!Serial.available()>0);
      deviceNumber = Serial.read() - '0'; 
      data = TransducerArray[deviceNumber-1].readSensor();
      Serial.println(data);
      dataFile.println(data);
    }
    else if(functionNumber == 4)
    {
      //Test SD Card data
      dataFile.println("The test Worked yeeeet");
    }
     else if(functionNumber == 5)
    {
      Serial.println("Which Solenoid would you like to Pulse?:");
      while(!Serial.available()>0);
      deviceNumber = Serial.read() - '0';
      
      if(deviceNumber>=1 && deviceNumber<7 )
      {
        if(SolenoidArray[deviceNumber-1].pulseSolenoid())
        {     
        Serial.print("Solenoid "+(String)deviceNumber+ " Status: ");
        Serial.println(SolenoidArray[deviceNumber-1].getSolenoidStatus());
        }
        else
        {
          Serial.println("Failed to Pulse Solenoid??");
        }
        
      }
      else
      {
        Serial.println("Invalid Device number");
      }
    }
    else if(functionNumber == 6)
    {
      Serial.println("Which Squib would you like to Fire?:");
      while(!Serial.available()>0);
      deviceNumber = Serial.read() - '0';
      
      if(deviceNumber>=1 && deviceNumber<8 )
      {
        if (deviceNumber == 1)
          ret = Squib_Fire(CMD_FIRE_1A,SquibA);
        if (deviceNumber == 2)
          ret = Squib_Fire(CMD_FIRE_1B,SquibA);
        if (deviceNumber == 3)
          ret = Squib_Fire(CMD_FIRE_2A,SquibA);
        if (deviceNumber == 4)
          ret = Squib_Fire(CMD_FIRE_2B,SquibA);
        if (deviceNumber == 5)
          ret = Squib_Fire(CMD_FIRE_1A,SquibB);
        if (deviceNumber == 6)
          ret = Squib_Fire(CMD_FIRE_1B,SquibB);
        if (deviceNumber == 7)
          ret = Squib_Fire(CMD_FIRE_2A,SquibB);
        if (deviceNumber == 8)
          ret = Squib_Fire(CMD_FIRE_2B,SquibB);

           Serial.println(ret);
      }
      else
      {
        Serial.println("Invalid Device number");
      }
    }
    else
    {
      Serial.println("Invalid Command");
    }
    Serial.print("Please Type Command:");
  }

 dataFile.flush();

  /*
  Serial.println((int)nitrousLine.readSensor());
  nitrousFill.openSolenoid();
  Serial.println(nitrousFill.getSolenoidStatus());
  delay(500);
 
  digitalWrite(LED_D3, HIGH);
  digitalWrite(LED_D2, LOW);
  nitrousFill.closeSolenoid();
  Serial.println(nitrousFill.getSolenoidStatus());

  delay(500);
  digitalWrite(LED_D3, LOW);
  digitalWrite(LED_D2, HIGH);
  */
}

extern "C"
{
  void debug(const char *data){
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
