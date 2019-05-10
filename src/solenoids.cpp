    #include <solenoids.hpp>
    #include <cstdint>
    #include <Arduino.h>
    using namespace std; 
    

    void Solenoids::initializeSolenoid(uint8_t solenoid_number)
    {
        this->solenoid_number =  solenoid_number;
        this->isOpen          = false;
        pinMode(this->solenoid_number, OUTPUT);
    };   
    
    int Solenoids::openSolenoid()
    {
        // Need to add Initital Pulse around a millisecond to open solenoid
        //digitalWrite(this->solenoid_number, HIGH);
        //delay(35);
        analogWrite(this->solenoid_number, (int)250); // PWM ~12% 30/255
        isOpen          = true;
        return 1; 

    };

    int Solenoids::closeSolenoid()
    {
        analogWrite(this->solenoid_number, (int)0);
        isOpen          = false;
        return 1;
    };

    bool Solenoids::getSolenoidStatus()
    {
        return isOpen;
    };

