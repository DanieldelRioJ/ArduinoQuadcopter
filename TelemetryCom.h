#ifndef TELEMETRY_COM
#define TELEMETRY_COM
#include "Arduino.h"
#include <IBusBM.h>

class TelemetryCom{
    public:
        TelemetryCom();
        void setup(HardwareSerial &serial);
        void loop();


    private:    
        void sendData();
        unsigned long lastUpdate;
        unsigned long voltage;
        IBusBM ibus;

        uint16_t rpm;
};

#endif