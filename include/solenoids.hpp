#include <cstdint>
using namespace std;

class Solenoids
{
    private:

    uint8_t solenoid_number;
    bool isOpen;
    
    public:

    void initializeSolenoid(uint8_t solenoid_number); 

    void openSolenoid();

    void closeSolenoid();

    bool getSolenoidStatus();

};