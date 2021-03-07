    #include <solenoids.hpp>
    #include <cstdint>
    #include <Arduino.h>
    using namespace std; 
    

    void Solenoids::initializeSolenoid(uint8_t solenoid_number, Solenoid_Size size, PULSE_Length pulse_time)
    {
        this->solenoid_number =  solenoid_number;
        this->size =  size;
        this->isOpen          = false;
        this->PWM       =   SOLENOID_PWM[size];
        this->PULSE_TIME = pulse_time;
        pinMode(this->solenoid_number, OUTPUT);
    };   
    
    int Solenoids::openSolenoid()
    {
        // Need to add Initital Pulse around a millisecond to open solenoid
        //digitalWrite(this->solenoid_number, HIGH);
        //delay(35);
        analogWrite(this->solenoid_number, this->PWM); // PWM ~12% 30/255
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
        return (int)isOpen;
    };

    int Solenoids::pulseSolenoid()

    {
        //Open for a set amount of time
        analogWrite(this->solenoid_number, (int)255);
        delay(PULSE_TIME);
        analogWrite(this->solenoid_number, (int)0);
        return 1;
    };

