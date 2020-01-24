#include <cstdint>
using namespace std;


// Bunch of Info From LoadCell Data Sheets


static const double LOAD_CELL_ADC_GAIN = 100; // For the love of god actaully measure this
static const double LOAD_CELL_RESPONSE = 0.003; //V/V 
static const double LOAD_CELL_MAX_LOAD = 1000; //V/V 
static const double LOAD_CELL_ADC_V = 3.3;
static const double LOAD_CELL_ADC_RES = 1024; // 10-Bit
// load_Cell_lbf = loadCell_raw * 3.3 * 1000 / (1024 * 50.1 * 0.003*12);
// Constant has everything but Battery Voltage which is read at time 
static const double LOAD_CELL_CONSTANT = LOAD_CELL_ADC_V*LOAD_CELL_MAX_LOAD/(LOAD_CELL_ADC_RES*LOAD_CELL_ADC_GAIN*LOAD_CELL_RESPONSE);

typedef enum {
    THRUST = 1,
    WEIGHT = 2
} LOAD_CELL_TYPE;

class LoadCell
{
    private:

    uint16_t sensor_number;
    double offset;
    LOAD_CELL_TYPE type;
    double VBATT;
    

    public:

    void initializeLoadCell(uint16_t sensor_number, LOAD_CELL_TYPE type,double VBATT); 

    // retuns the int of raw ADC Bin 
    int readLoadCellRaw();

    //returns load value in lbf
    double readLoadCell();

    // averages X Samples to get a baseline, returns an offset value
    double offsetCalibrationSequence();

    // Calculate Resolution of the Load Cell/ ADC Combo
    double calculateResolution();

};