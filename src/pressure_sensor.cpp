    #include <pressure_sensor.hpp>
    #include <cstdint>
    #include <Arduino.h>
    using namespace std; 
    
    
    void PressureSensor::initializeSensor(uint16_t sensor_number)
    {
        this->sensor_number =  sensor_number;
        pinMode(this->sensor_number, INPUT);
    }; 

    int PressureSensor::readSensor()
    {
        return analogRead(this->sensor_number)/(this->scaler);
    };






