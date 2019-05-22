#include <cstdint>
using namespace std;

static const double PRESSURE_SCALER_1K = 0.0045;
static const double PRESSURE_SCALER_2K = 0.0018;//Fill this in

typedef enum {
    RANGE_1K = 1,
    RANGE_2K = 2
} PRESSURE_DUCER_RANGE;

class PressureSensor
{
    private:

    uint16_t sensor_number;
    double scaler;
    PRESSURE_DUCER_RANGE range;
    signed int biasOffset;

    public:

    void initializeSensor(uint16_t sensor_number, PRESSURE_DUCER_RANGE range, int biasOffset); 

    double readSensor();

};