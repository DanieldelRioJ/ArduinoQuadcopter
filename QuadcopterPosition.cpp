#include "QuadcopterPosition.h"
#include "Arduino.h"

void QuadcopterPosition::setYPR(float ypr[3]){
    this->ypr[0] = ypr[0];
    this->ypr[1] = ypr[1];
    this->ypr[2] = ypr[2];
    this->updateTime = millis();
}