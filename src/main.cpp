#include <Arduino.h>
#include <pin_interface.h>
#include <solenoids.hpp>
#include <pressure_sensor.hpp>
#include  "SD.h"
#include "MC33797.h"
// #include "ADS1148.h"
#include "ads1248.h"
#include <SPI.h>
#include "wiring_private.h"
//#include <datalogger.hpp>

Solenoids oxidizerTankVent, nitrogenFill, nitrousFill, nitrousAbort, nitrogenAbort, pyrovalveShutOff;
PressureSensor nitrousLine, nitrousHeatXger, nitrogenLine, oxidizerTank, combustionChamber;
File dataFile;
double data;

Solenoids SolenoidArray[]          = {oxidizerTankVent, nitrogenFill, nitrousFill, nitrousAbort, nitrogenAbort, pyrovalveShutOff}; // J12, J13, J15, J16, J19, J20
PressureSensor TransducerArray[]   = {nitrousLine, nitrousHeatXger, nitrogenLine, oxidizerTank, combustionChamber}; 

const uint8_t SquibA = 1;
const uint8_t SquibB = 2;
unsigned long startTime = millis();

int functionNumber  =  -1;
int deviceNumber = -1;

SPIClass squibSPI (&sercom0, Squib_MISO, Squib_SCK, Squib_MOSI, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_2);
SPIClass Load_Cell_SPI (&sercom2, Load_Cell_MISO, Load_Cell_SCK, Load_Cell_MOSI, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_3);

//Want to move this to sepreate File
int PrintTansducerValuesSerial(PressureSensor transducers[], unsigned long timeMillis);
// Should log to SD too with time stamps 
int LogTansducerValuesSD(PressureSensor transducers[], unsigned long timeMillis);
//Creates Packets to send to Print and Log functions
String createDataPacket(PressureSensor transducers[], unsigned long timeMillis);

void setup() {

  // put your setup code here, to run once:
  //pinMode(Solenoid_1, OUTPUT);
  Serial.begin(115200);
  delay(4000);
  Serial.println("Setup Begin");
  pinMode(LED_D2, OUTPUT);
  pinMode(LED_D3, OUTPUT);

  TransducerArray[0].initializeSensor(Pressure_1,RANGE_1K,3); // Nitrous Line/Supply 3
  TransducerArray[1].initializeSensor(Pressure_2,RANGE_1K,7); // Nitrous after Heat Exchanger 7
  TransducerArray[2].initializeSensor(Pressure_3,RANGE_1K,0); // Nitrogen Line 6
  TransducerArray[3].initializeSensor(Pressure_4,RANGE_2K,-5); // Oxidizier Tank (On Motor) 0
  TransducerArray[4].initializeSensor(Pressure_5,RANGE_2K,-2); // Combustion Chamber or Manifold Pressure -9

  SolenoidArray[0].initializeSolenoid(Solenoid_1, SOLENOID_MEDIUM); // Oxidizer Tank Vent, On Rocket, Edelbrook (J12)
  SolenoidArray[1].initializeSolenoid(Solenoid_2, SOLENOID_MEDIUM); // Nitrogen Fill (J13)
  SolenoidArray[2].initializeSolenoid(Solenoid_3, SOLENOID_LARGE); //  Nitrous Fill, Pro BigShot?? (J15)
  SolenoidArray[3].initializeSolenoid(Solenoid_4, SOLENOID_SMALL); //  Nitrous A132312112bort (J16)
  SolenoidArray[4].initializeSolenoid(Solenoid_5, SOLENOID_SMALL); //  Nitrogen Abort (J19)
  SolenoidArray[5].initializeSolenoid(Solenoid_6, SOLENOID_SMALL); // Pyrovalve Shut Off(J20)

  pinPeripheral(Squib_MISO, PIO_SERCOM_ALT);
  pinPeripheral(Squib_SCK, PIO_SERCOM_ALT);
  pinPeripheral(Squib_MOSI, PIO_SERCOM_ALT);

  pinMode(Squib_SS_1, OUTPUT);
  pinMode(Squib_SS_2, OUTPUT);
  digitalWrite(Squib_SS_1, HIGH);
  digitalWrite(Squib_SS_2, HIGH);
  squibSPI.begin();

  pinMode(Load_Cell_SS, OUTPUT);
  digitalWrite(Load_Cell_SS, HIGH);
  Load_Cell_SPI.begin();
  
  Serial.println("opening ADS1148");
  // openADS1148();
  ADS1148_Quail_Config();
  delay(1000);
  Serial.println("done opening ADS1148");
  
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
  dataFile = SD.open("datalog52419.txt", FILE_WRITE);
  if (!dataFile) {
    Serial.println("error opening datalog.txt");
    // Wait forever since we cant write data
     while (!dataFile) {
        Serial.println("error opening datalog.txt");
        delay(500);
        dataFile = SD.open("datalogl.txt", FILE_WRITE);

     }
  }
  dataFile.println("Data logging for Quail has begun yeet");
  Serial.println("Setup done");
  
  
}

void loop() {
  // put your main code here, to run repeatedly:
 
  //Squib_StatusType *s = new Squib_StatusType();

  uint8_t ret;
  unsigned long currTime = millis();
  String data;
  
 //Serial.println(currTime);


  if((currTime-5 ) > startTime)
  {
          data = createDataPacket(TransducerArray, currTime);
          //PrintTansducerValuesSerial(TransducerArray, currTime);
          Serial.println(data);
          dataFile.println(data);
          startTime = currTime;
  }


  if (Serial.available() > 0)
  {
    if(functionNumber == -1)
    {
      functionNumber = Serial.read() - '0';
    }
    else
    {
      deviceNumber = Serial.read() - '0';
    } 

    Serial.println("");
  }

     /* 
      subtract 1 from the recipe number to get the index in the array
    */ 
   
   

   if(functionNumber >0 && deviceNumber>0)
   {
    
    if(functionNumber == 1)
    {
      // Function 1 OPENS Solenoid and prints status
      //Serial.println("Which Solenoid would you like to Open?:");
      //while(!Serial.available()>0);
      
      //deviceNumber = Serial.read() - '0';
      
      if(deviceNumber>=1 && deviceNumber<7 )
      {
        if(SolenoidArray[deviceNumber-1].openSolenoid())
        {     
        //Serial.print("Solenoid "+(String)deviceNumber+ " Status: ");
        //Serial.println(SolenoidArray[deviceNumber-1].getSolenoidStatus());
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
      functionNumber = -1;
      deviceNumber = -1;
      }
    else if(functionNumber == 2)
    {
      // Function 2 CLOSES Solenoid and prints status
      //Serial.println("Which Solenoid would you like to Close?:");
      //while(!Serial.available()>0);
      // deviceNumber = Serial.read() - '0';

      if(deviceNumber>=1 && deviceNumber<7 )
      {
        SolenoidArray[deviceNumber-1].closeSolenoid();
        //Serial.print("Solenoid "+(String)deviceNumber+ " Status: ");
        //Serial.println(nitrousFill.getSolenoidStatus());
      }
      else
      {
        Serial.println("Invalid Device number");
      }
      functionNumber = -1;
      deviceNumber = -1;
      
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
      //dataFile.println("The test Worked yeeeet");
      dataFile.println(createDataPacket(TransducerArray, currTime));
      //PrintTansducerValuesSerial(TransducerArray, currTime);
      functionNumber = -1;
      deviceNumber = -1;
    }
     else if(functionNumber == 5)
    {
      //Serial.println("Which Solenoid would you like to Pulse?:");
      //while(!Serial.available()>0);
      //deviceNumber = Serial.read() - '0';
      
      if(deviceNumber>=1 && deviceNumber<7 )
      {
        if(SolenoidArray[deviceNumber-1].pulseSolenoid())
        {     
        //Serial.print("Solenoid "+(String)deviceNumber+ " Status: ");
        //Serial.println(SolenoidArray[deviceNumber-1].getSolenoidStatus());
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
      functionNumber = -1;
      deviceNumber = -1;
    }
    else if(functionNumber == 6)
    {
      //Serial.println("Which Squib would you like to Fire?:");
      //while(!Serial.available()>0);
      //deviceNumber = Serial.read() - '0';
      
      if(deviceNumber>=1 && deviceNumber<10 )
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
        if (deviceNumber == 9){
          ret = Squib_Fire(CMD_FIRE_1A,SquibA);
          ret = Squib_Fire(CMD_FIRE_1A,SquibB);
        }
           //Serial.println(ret);
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
    //Serial.print("Please Type Command:");
   }

 dataFile.flush();

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

  uint8_t send_load_cell(uint8_t data)
  {
    // delay(10);
    // digitalWrite(Load_Cell_SS, LOW);
    Load_Cell_SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    uint8_t val = Load_Cell_SPI.transfer(data);
    Load_Cell_SPI.endTransaction();
    // digitalWrite(Load_Cell_SS, HIGH);
    return val;
  }

  void start_load_cell_transaction() {
    Load_Cell_SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
  }

  void stop_load_cell_transaction() {
    Load_Cell_SPI.endTransaction();
  }

  uint8_t send_load_cell_byte_many(uint8_t data) {
    return Load_Cell_SPI.transfer(data);
  }

  void print_ADS1148_init(uint8_t out[], uint8_t len) {
    Serial.println("Load Cell ADC initialization bytes:");
    for (uint8_t i = 0; i<len; i++) {
      Serial.println(out[i], HEX);
    }
  }

  void print_byte_with_desc(char* string, uint8_t byte_out) {
    Serial.print(string);
    Serial.println(byte_out, HEX);
  }
  void load_cell_assert_CS(int fAssert) {
    if (fAssert) {
      delayMicroseconds(2);   // wait minimum of 7 clock periods (1 / 4.096 MHz) before bringing CS back up
      digitalWrite(Load_Cell_SS, HIGH);
    } else
      digitalWrite(Load_Cell_SS, LOW);
  }
}



String createDataPacket(PressureSensor transducers[], unsigned long timeMillis){
    
    String out = "0, ";
    
      
      for(int i =0; i<5; i++)
    {
       data = transducers[i].readSensor();

        if (data>2000 || data<-200)
            data = -1;

        out = out + data +",";
       
    }
        
       out = out + timeMillis +",";

return out;
}


int PrintTansducerValuesSerial(PressureSensor transducers[], unsigned long timeMillis)
{
    //Serial.println("Sending Data:");

    for(int i =0; i<5; i++)
    {
       data = transducers[i].readSensor();

        if (data>2000 || data<-200)
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

    for(int i =0; i<5; i++)
    {
       data = transducers[i].readSensor();

        if (data>2000 || data<-200)
            data = -1;

        dataFile.print(data);
        dataFile.print(",");
       
    }
        
        Serial.print(",");
        Serial.println("0");

return 1;
}
