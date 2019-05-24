#include <cstdint>
using namespace std;


typedef enum{
    SOLENOID_SMALL  = 0,
    SOLENOID_MEDIUM = 1,
    SOLENOID_LARGE = 2
}Solenoid_Size;

typedef enum{
    MILLIS_100 = 100,
    MILLIS_200 = 200,
    MILLIS_500 = 500,
    SECONDS_1  = 1000,
    SECONDS_2  = 2000, 
    SECONDS_5  = 5000,
    SECONDS_10 = 10000
}PULSE_Length;



static const uint16_t PULSE_TIME = 100;

//From Tests
static const uint8_t SMALL_PWM  = 255 ;
static const uint8_t MEDIUM_PWM = 255;
static const uint8_t LARGE_PWM = 255;
static const uint8_t SOLENOID_PWM[3] = {SMALL_PWM, MEDIUM_PWM, LARGE_PWM};

class Solenoids
{
    private:

    uint8_t solenoid_number;
    uint8_t PWM;    //This depends on the size of solenoid, minimum to keep it open 
    char16_t myName;    //probably should have a name
    bool isOpen;
    Solenoid_Size size;
    
    public:

    void initializeSolenoid(uint8_t solenoid_number,Solenoid_Size size ); 


    // returns 1 if it does the thing
    int openSolenoid();

    // returns 1 if it does the thing
    int closeSolenoid();

    //Returns 1 if open 0 if closed
    bool getSolenoidStatus();

    //Pulses for default = PULSE_TIME
    int pulseSolenoid();

    //Pulses for specified time
    int pulseSolenoid(PULSE_Length time);

};