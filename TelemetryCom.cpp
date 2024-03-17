#include "TelemetryCom.h"
#include "iBUSSensors.h"

#define TEMPBASE 400  

TelemetryCom::TelemetryCom(){
    this->lastUpdate = 0;
    this->rpm = 0;
}

void TelemetryCom::setup(HardwareSerial &serial){    
    this->ibus.begin(serial, IBUSBM_NOTIMER);
    this->ibus.addSensor(IBUSS_RPM);
    this->ibus.addSensor(IBUS_MEAS_TYPE_EXTV);    
    this->ibus.addSensor(IBUS_MEAS_TYPE_TEM);    
    this->ibus.addSensor(IBUS_MEAS_TYPE_BAT_CURR);    
    this->ibus.addSensor(IBUS_PRESS, 4);
    this->sendData();
}

void TelemetryCom::loop(){
    this->ibus.loop();
    const unsigned long now = millis();
    if(now - this->lastUpdate > 500){        
        this->rpm++;
        this->lastUpdate = now;
        this->sendData();
    }
}

void TelemetryCom::sendData(){
    this->voltage = this->voltage * 0.9 + 0.1 * (float)analogRead(A0) / 1024 * 5.3 * 7.85 * 100;
    this->ibus.setSensorMeasurement(1,this->rpm);        
    this->ibus.setSensorMeasurement(2,this->voltage);         
    this->ibus.setSensorMeasurement(3,TEMPBASE + 150);        
    this->ibus.setSensorMeasurement(4, 100);  
    this->ibus.setSensorMeasurement(5, 100000);
}


