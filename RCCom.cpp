#include "RCCom.h"

void RCCom::setup(HardwareSerial &serial){
    this->ibus.begin(serial, IBUSBM_NOTIMER);
}

void RCCom::loop(){        
    this->ibus.loop();
    this->roll = this->ibus.readChannel(0);
    this->pitch = this->ibus.readChannel(1);
    this->throttle = this->ibus.readChannel(2);
    this->yaw = this->ibus.readChannel(3);
    this->p1 = this->ibus.readChannel(4);
    this->p2 = this->ibus.readChannel(5);
    this->intA = this->ibus.readChannel(6);
    this->intB = this->ibus.readChannel(7);
    this->intC = this->ibus.readChannel(8);
    this->intD = this->ibus.readChannel(9);
}


