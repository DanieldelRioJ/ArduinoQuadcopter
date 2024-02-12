#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "TelemetryCom.h"
#include "RCCom.h"
#include "Engine.h"
#include <PID_v1.h>

#define ENGINE_FR 7
#define ENGINE_FL 6
#define ENGINE_RR 5
#define ENGINE_RL 4


TelemetryCom telemetryCom;
RCCom rcCom;
Engine engineFR;
Engine engineFL;
Engine engineRR;
Engine engineRL;

MPU6050 mpu;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
float lastYaw = 0;
float yawSpeed = 0;

double Kp=0.4, Ki=0.002, Kd=0.04;
double KpYaw=8, KiYaw=0.1, KdYaw=0.04;
double pitchCorrection = 0, rollCorrection = 0, yawCorrection = 0;
PID pidPitch((double*)&ypr[1], &pitchCorrection, &(rcCom.pitch), Kp, Ki, Kd, P_ON_E, DIRECT);
PID pidRoll((double*)&ypr[2], &rollCorrection, &(rcCom.roll), Kp, Ki, Kd, P_ON_E, DIRECT);
PID pidYaw((double*)&yawSpeed, &yawCorrection, &(rcCom.yaw), KpYaw, KiYaw, KdYaw, P_ON_E, DIRECT);

void setup() {

    Wire.begin();
    Wire.setClock(400000);
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.setXAccelOffset(712);
    mpu.setYAccelOffset(-25);
    mpu.setZAccelOffset(1105);
    mpu.setXGyroOffset(90);
    mpu.setYGyroOffset(-12);
    mpu.setZGyroOffset(37);
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    fifoCount = mpu.getFIFOCount();

    Serial.begin(115200);
    telemetryCom.setup(Serial2);    
    rcCom.setup(Serial1);
    engineFR.setup(ENGINE_FR);
    engineFL.setup(ENGINE_FL);
    engineRR.setup(ENGINE_RR);
    engineRL.setup(ENGINE_RL);
    _pidSetup();
}

uint16_t hz = 0;
unsigned long lastHzPrint = 0;
bool newAttitudeData = false;
void loop() {
    /*while(true){
        _loop();
    }*/
    while (fifoCount < packetSize) {
        _loop();  
        fifoCount = mpu.getFIFOCount();

    }
    if (fifoCount == 1024) {    
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));        
    }
    else{    
        if (fifoCount % packetSize != 0) {
            Serial.println(F("Reset!"));
            mpu.resetFIFO();            
        }
        else{    
            while (fifoCount >= packetSize) {            
                mpu.getFIFOBytes(fifoBuffer,packetSize);
                fifoCount -= packetSize;                
            }    
            unsigned long now = millis();
            hz++;
            if(now - lastHzPrint > 1000){
                lastHzPrint = now;
                Serial.print("HZ = ");
                Serial.println(hz);
                hz = 0;
            }
            newAttitudeData = true;
        
            mpu.dmpGetQuaternion(&q,fifoBuffer);
            mpu.dmpGetGravity(&gravity,&q);
            mpu.dmpGetYawPitchRoll(ypr,&q,&gravity);          
            ypr[0] = ypr[0]*180/PI;
            ypr[1] = ypr[1]*180/PI;
            ypr[2] = ypr[2]*180/PI;
            yawSpeed = ypr[0] - lastYaw;
            lastYaw = ypr[0];

            /*Serial.print("ypr\t");
            Serial.print(ypr[0]);
            Serial.print("\t");
            Serial.print(ypr[1]);
            Serial.print("\t");
            Serial.print(ypr[2]);
            Serial.print("\t");
            Serial.print(yawSpeed);
            Serial.println();*/
            
        }   
    }

}

void _loop(){
    telemetryCom.loop();
    rcCom.loop();    

    if(newAttitudeData){
        newAttitudeData = false;
        _pidLoop();
        Serial.print("pitchCorrection = ");
        Serial.print(pitchCorrection);
        Serial.print("\trollCorrection = ");
        Serial.print(rollCorrection);
        Serial.print("\tYawCorrection = ");
        Serial.print(yawCorrection);
        Serial.print("\trollCorrection = ");
        Serial.print(yawSpeed);
        Serial.print("\trollCorrection = ");
        Serial.println(((double)rcCom.p2 - 1000) / 10000);
        _changeTunnings(((double)rcCom.p1 - 1000) / 1000, ((double)rcCom.p2 - 1000) / 10000);

        if(rcCom.intD == 1000){
            engineFR.setPower(rcCom.throttle);
            engineFL.setPower(rcCom.throttle);
            engineRR.setPower(rcCom.throttle);
            engineRL.setPower(rcCom.throttle);
        }else {            
            engineFR.setPower(rcCom.throttle + rollCorrection - pitchCorrection + yawCorrection);
            engineFL.setPower(rcCom.throttle - rollCorrection - pitchCorrection - yawCorrection);
            engineRR.setPower(rcCom.throttle + rollCorrection + pitchCorrection - yawCorrection);
            engineRL.setPower(rcCom.throttle - rollCorrection + pitchCorrection + yawCorrection);
        }
    }
}

void _pidSetup(){   
    
    pidPitch.SetMode(AUTOMATIC);
    pidPitch.SetSampleTime(10);
    pidPitch.SetOutputLimits(-4,4); //-% and +% motor speed correction effect

    pidRoll.SetMode(AUTOMATIC);
    pidRoll.SetSampleTime(10);
    pidRoll.SetOutputLimits(-4,4); //-% and +% motor speed correction effect    

    pidYaw.SetMode(AUTOMATIC);
    pidYaw.SetSampleTime(10);
    pidYaw.SetOutputLimits(-10,10); //-% and +% motor speed correction effect
}

void _changeTunnings(double rcKp, double rcKd){
    pidPitch.SetTunings(rcKp, Ki, rcKd);
    pidRoll.SetTunings(rcKp, Ki, rcKd);
    //pidYaw.SetTunings(rcKp*10, rcKd, KdYaw);
}

void _pidLoop(){
    pidPitch.Compute();
    pidRoll.Compute();
    pidYaw.Compute();
}