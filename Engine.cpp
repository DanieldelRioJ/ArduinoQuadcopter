#include "Engine.h"
#include "Arduino.h"


void Engine::setup(uint8_t pin){    
    this->_esc.attach(pin, 1000, 2000);
};

void Engine::setPower(int power){    
    this->_esc.writeMicroseconds(constrain(power, 1000, 2000));
    //Serial.println(constrain(power, 1000, 2000));
};