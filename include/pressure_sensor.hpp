#include <cstdint>
using namespace std;

static const double PRESSURE_SCALER = 0.0045;

class PressureSensor
{
    private:

    uint16_t sensor_number;
    double scaler = PRESSURE_SCALER;

    public:

    void initializeSensor(uint16_t sensor_number); 

    double readSensor();

};