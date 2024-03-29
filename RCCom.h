#ifndef RC_COM
#define RC_COM
#include "Arduino.h"
#include <IBusBM.h>

class RCCom{
    public:
        void loop();
        void setup(HardwareSerial &serial);
   
        double pitch;
        double roll;    
        double yaw;         
        float throttle;        
        uint16_t p1;
        uint16_t p2;
        
        uint16_t intA;
        uint16_t intB;
        uint16_t intC;
        uint16_t intD;

    private:    
        IBusBM ibus;

};

#endif