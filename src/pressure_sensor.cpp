    #include <pressure_sensor.hpp>
    #include <cstdint>
    #include <Arduino.h>
    using namespace std; 
    
    
    void PressureSensor::initializeSensor(uint16_t sensor_number, PRESSURE_DUCER_RANGE range, signed int biasOffset)
    {
        this->sensor_number =  sensor_number;
        pinMode(this->sensor_number, INPUT);

        this->range = range;
        this->biasOffset = biasOffset;

        if(range == RANGE_1K)
        this->scaler = PRESSURE_SCALER_1K;
        else
        this->scaler = PRESSURE_SCALER_2K;
        

    }; 

    double PressureSensor::readSensor()
    {
        //return ((double)(analogRead(this->sensor_number))*(2*3.3/1023)-.5)*(this->scaler);
        double raw = (double)(analogRead(this->sensor_number)+this->biasOffset);
        return  ((raw*2*3.3/1023)-.5)/(this->scaler);


    };






