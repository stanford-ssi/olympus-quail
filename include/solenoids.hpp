#include <cstdint>
using namespace std;

class Solenoids
{
    private:

    uint8_t solenoid_number;
    uint8_t PWM;    //This depends on the size of solenoid, minimum to keep it open 
    char16_t myName;    //probably should have a name
    bool isOpen;
    
    public:

    void initializeSolenoid(uint8_t solenoid_number); 

    void openSolenoid();

    void closeSolenoid();

    bool getSolenoidStatus();

};