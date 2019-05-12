#include <cstdint>
using namespace std;

static const uint8_t SMALL  = 1;
static const uint8_t MEDIUM = 2;
static const uint8_t LARGE = 3;
static const uint16_t PULSE_TIME = 1000   ;

//From Tests
static const uint8_t SMALL_PWM  =80 ;
static const uint8_t MEDIUM_PWM = 80;
static const uint8_t LARGE_PWM = 100;
static const uint8_t SOLENOID_PWM[3] = {SMALL_PWM, MEDIUM_PWM, LARGE_PWM};

class Solenoids
{
    private:

    uint8_t solenoid_number;
    uint8_t PWM;    //This depends on the size of solenoid, minimum to keep it open 
    char16_t myName;    //probably should have a name
    bool isOpen;
    uint8_t size;
    
    public:

    void initializeSolenoid(uint8_t solenoid_number,uint8_t size ); 


    // returns 1 if it does the thing
    int openSolenoid();

    // returns 1 if it does the thing
    int closeSolenoid();

    bool getSolenoidStatus();

    int pulseSolenoid();
};