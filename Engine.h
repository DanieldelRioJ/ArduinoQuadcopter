#ifndef Engine_H
#define Engine_H

#include <Servo.h>

class Engine{
    public:   
        void setup(uint8_t pin); 
        void setPower(int power); //0 to 100%

    private:
        Servo _esc;        
        unsigned long lastUpdate;
};

#endif