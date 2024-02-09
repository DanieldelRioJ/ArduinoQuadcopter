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
    mpu.setXAccelOffset(698);
    mpu.setYAccelOffset(-15);
    mpu.setZAccelOffset(1110);
    mpu.setXGyroOffset(89);
    mpu.setYGyroOffset(-11);
    mpu.setZGyroOffset(41);
    mpu.setDMPEnabled(true);
    if (mpu.testConnection()) Serial.println("Sensor iniciado correctamente");
    else Serial.println("Error al iniciar el sensor");

    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();
};

void MyMPU::refreshFifoCount(){
    fifoCount = mpu.getFIFOCount();
}

bool MyMPU::loop(){

    if(fifoCount < packetSize){
        this->refreshFifoCount();
        return false;
    }

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
                mpu.getFIFOBytes(fifoBuffer,packetSize);
                fifoCount -= packetSize;
                
            }    
            
            //Serial.println(F("Get data!"));
            mpu.dmpGetQuaternion(&q,fifoBuffer);
            mpu.dmpGetGravity(&gravity,&q);
            mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);  
            //this->quadcopterPosition.setYPR(ypr);       
            
            /*Serial.print("ypr\t");
            Serial.print(ypr[0]*180/PI);
            Serial.print("\t");
            Serial.print(ypr[1]*180/PI);
            Serial.print("\t");
            Serial.print(ypr[2]*180/PI);
            Serial.println(); */
            return true;
        }   
   
    }
    return false;
}