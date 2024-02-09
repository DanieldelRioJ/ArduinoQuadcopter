#include "RCCom.h"

#define MAX_ANGLE 15
#define MAX_ANGLE_SPEED 2 //PER READ

void RCCom::setup(HardwareSerial &serial){
    this->ibus.begin(serial, IBUSBM_NOTIMER);
}

void RCCom::loop(){        
    this->ibus.loop();
    this->roll = -2 * MAX_ANGLE * ((float)this->ibus.readChannel(0)-1500)/1000;
    this->pitch = 2 * MAX_ANGLE * ((float)this->ibus.readChannel(1)-1500)/1000;
    this->throttle = ((float)this->ibus.readChannel(2) - 1000) / 10;
    this->yaw = 2 * MAX_ANGLE_SPEED * ((float)this->ibus.readChannel(3) - 1500) / 1000;
    this->p1 = this->ibus.readChannel(4);
    this->p2 = this->ibus.readChannel(5);
    this->intA = this->ibus.readChannel(6);
    this->intB = this->ibus.readChannel(7);
    this->intC = this->ibus.readChannel(8);
    this->intD = this->ibus.readChannel(9);
}


