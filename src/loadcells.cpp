    #include <loadcells.hpp>
    #include <cstdint>
    #include <Arduino.h>
    using namespace std; 
    

    void LoadCell::initializeLoadCell(uint16_t sensor_number,  LOAD_CELL_TYPE type, double VBATT)
    {
        this->sensor_number =  sensor_number;
        this->type = type;
        this->offset = 0;
        this->VBATT = VBATT;
        pinMode(this->sensor_number, INPUT);
    };   
    

    // retuns the int of raw ADC Bin 
    int LoadCell::readLoadCellRaw()
    {
        return analogRead(this->sensor_number);

    };

    //returns load value in lbf
    double LoadCell::readLoadCell()
    {
        // Global for Battery Voltage

        // loadCell_raw * 3.3 * 1000 / (1024 * 50.1 * 0.003*12);
        return (double)(this->readLoadCellRaw()-this->offset)*LOAD_CELL_CONSTANT/this->VBATT;
    };
    // averages X Samples to get a baseline, returns an offset value
    double LoadCell::offsetCalibrationSequence()
    {
        // record X Samples (raw Bins)

        return 0;

    };

