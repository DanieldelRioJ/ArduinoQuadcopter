#include "Engine.h"
#include "Arduino.h"


void Engine::setup(uint8_t pin){    
    this->_esc.attach(pin, 1000, 2000);
};

void Engine::setPower(float power){ 
    float val = constrain(power, 0, 100) * 10 + 1000;
    this->_esc.writeMicroseconds(val);
    //Serial.println(val);
};