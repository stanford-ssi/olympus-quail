#include <cstdint>
using namespace std;

class PressureSensor
{
    private:

    uint16_t sensor_number;
    uint16_t scaler = 1;

    public:

    void initializeSensor(uint16_t sensor_number); 

    int readSensor();

};