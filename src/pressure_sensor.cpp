    #include <pressure_sensor.hpp>
    #include <cstdint>
    #include <Arduino.h>
    using namespace std; 
    
    
    void PressureSensor::initializeSensor(uint16_t sensor_number)
    {
        this->sensor_number =  sensor_number;
        pinMode(this->sensor_number, INPUT);
    }; 

    double PressureSensor::readSensor()
    {
        //return ((double)(analogRead(this->sensor_number))*(2*3.3/1023)-.5)*(this->scaler);

        return ((double)(analogRead(this->sensor_number)))*((2*3.3/1023)-.5)*(this->scaler);


    };






