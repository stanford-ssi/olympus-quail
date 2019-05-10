#include <cstdint>
using namespace std;

class PressureSensor
{
    private:

    uint16_t sensor_number;
    double scaler;

    public:

    void initializeSensor(uint16_t sensor_number); 

    double readSensor();

};