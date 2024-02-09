#ifndef MyMpu_h
#define MyMpu_h

#include "MPU6050_6Axis_MotionApps20.h"
#include "QuadcopterPosition.h"

class MyMPU {
    public:       
        MyMPU(QuadcopterPosition quadcopterPosition);

        QuadcopterPosition quadcopterPosition;
        MPU6050 mpu;
        uint16_t packetSize;
        uint16_t fifoCount;
        uint8_t fifoBuffer[64];
        Quaternion q;
        VectorFloat gravity;
        float ypr[3];

        void setup();
        bool loop();
        void refreshFifoCount();
};

#endif