#include <Arduino.h>
#include <pin_interface.h>
#include <solenoids.hpp>
#include <pressure_sensor.hpp>
#include  "SD.h"


Solenoids oxidizerTankVent, nitrogenFill, nitrousFill, nitrousAbort, nitrogenAbort, pyrovalveShutOff;
PressureSensor nitrousLine, nitrousHeatXger, nitrogenLine, oxidizerTank, combustionChamber;
File dataFile;
uint16_t data;

Solenoids SolenoidArray[]          = {oxidizerTankVent, nitrogenFill, nitrousFill, nitrousAbort, nitrogenAbort, pyrovalveShutOff}; // J12, J13, J15, J16, J19, J20
PressureSensor TransducerArray[]   = {nitrousLine, nitrousHeatXger, nitrogenLine, oxidizerTank, combustionChamber}; 


void setup() {
  // put your setup code here, to run once:
  //pinMode(Solenoid_1, OUTPUT);
  pinMode(LED_D2, OUTPUT);
  pinMode(LED_D3, OUTPUT);

  TransducerArray[0].initializeSensor(Pressure_1);
  TransducerArray[1].initializeSensor(Pressure_2);
  TransducerArray[2].initializeSensor(Pressure_3);
  TransducerArray[3].initializeSensor(Pressure_4);
  TransducerArray[4].initializeSensor(Pressure_5);

  SolenoidArray[0].initializeSolenoid(Solenoid_1);
  SolenoidArray[1].initializeSolenoid(Solenoid_2);
  SolenoidArray[2].initializeSolenoid(Solenoid_3);
  SolenoidArray[3].initializeSolenoid(Solenoid_4);
  SolenoidArray[4].initializeSolenoid(Solenoid_5);
  SolenoidArray[5].initializeSolenoid(Solenoid_6);


  Serial.begin(9600);
  Serial.println("Setup done");
  //digitalWrite(LED_D2, HIGH);
  digitalWrite(LED_D3, HIGH);
  delay(10);

  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SD_Card_SS, OUTPUT);
  
  // see if the card is present and can be initialized:
  if (!SD.begin(SD_Card_SS, SD_Card_MOSI, SD_Card_MISO, SD_Card_SCK)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1) ;
  }
  Serial.println("card initialized.");
  
  // Open up the file we're going to log to!
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (! dataFile) {
    Serial.println("error opening datalog.txt");
    // Wait forever since we cant write data
    while (1) ;
  }
  dataFile.println("Data logging for Quail has begun yeet");

}

void loop() {
  // put your main code here, to run repeatedly:
 
  int functionNumber  =  -1;
  int deviceNumber =-1;

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

      if(deviceNumber>=1 && deviceNumber<6 )
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