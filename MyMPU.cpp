#include "MyMPU.h"
#include "I2Cdev.h"
#include "Wire.h"
#include "Arduino.h"

MyMPU::MyMPU(QuadcopterPosition quadcopterPosition){
    this->quadcopterPosition = quadcopterPosition;
}

void MyMPU::setup(){

    Wire.begin();
    TWBR = 24;
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXAccelOffset(-1878);
    mpu.setYAccelOffset(-4662);
    mpu.setZAccelOffset(407);
    mpu.setXGyroOffset(97);
    mpu.setYGyroOffset(-37);
    mpu.setZGyroOffset(-24);
    mpu.setDMPEnabled(true);
    if (mpu.testConnection()) Serial.println("Sensor iniciado correctamente");
    else Serial.println("Error al iniciar el sensor");

    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();
};

void MyMPU::refreshFifoCount(){
    fifoCount = mpu.getFIFOCount();
}

void MyMPU::loop(){
    if (fifoCount == 1024) {
    
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
        
    }
    else{
    
        if (fifoCount % packetSize != 0) {
            
            Serial.println(F("FIFO overflow!"));
            mpu.resetFIFO();
            
        }
        else{
    
            while (fifoCount >= packetSize) {
                Serial.println(F("Reset mpu!"));
                mpu.getFIFOBytes(fifoBuffer,packetSize);
                fifoCount -= packetSize;
                
            }    
            
            //Serial.println(F("Get data!"));
            mpu.dmpGetQuaternion(&q,fifoBuffer);
            mpu.dmpGetGravity(&gravity,&q);
            mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);   
            //this->quadcopterPosition.setYPR(ypr);       
            
            Serial.print("ypr\t");
            Serial.print(ypr[0]*180/PI);
            Serial.print("\t");
            Serial.print(ypr[1]*180/PI);
            Serial.print("\t");
            Serial.print(ypr[2]*180/PI);
            Serial.println();   
        }   
   
    }
}