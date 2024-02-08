/*
// Librerias I2C para controlar el mpu6050
// la libreria MPU6050.h necesita I2Cdev.h, I2Cdev.h necesita Wire.h
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z
int ax, ay, az;
int gx, gy, gz;

void setup() {
  Serial.begin(57600);    //Iniciando puerto serial
  Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");
}

void loop() {
  // Leer las aceleraciones y velocidades angulares
  sensor.getAcceleration(&ax, &ay, &az);
  sensor.getRotation(&gx, &gy, &gz);

  //Mostrar las lecturas separadas por un [tab]
  Serial.print("a[x y z] g[x y z]:\t");
  Serial.print(ax); Serial.print("\t");
  Serial.print(ay); Serial.print("\t");
  Serial.print(az); Serial.print("\t");
  Serial.print(gx); Serial.print("\t");
  Serial.print(gy); Serial.print("\t");
  Serial.println(gz);

  delay(100);
}*/
#include "TelemetryCom.h"
#include "RCCom.h"
#include "Engine.h"
#include "QuadcopterPosition.h"
#include "MyMPU.h"

#define ENGINE_FR 4
#define ENGINE_FL 5
#define ENGINE_RR 6
#define ENGINE_RL 7


TelemetryCom telemetryCom;
RCCom rcCom;
Engine engineFR;
Engine engineFL;
Engine engineRR;
Engine engineRL;
QuadcopterPosition quadcopterPosition;
MyMPU myMPU(quadcopterPosition);

void setup(){
    Serial.begin(115200);
    telemetryCom.setup(Serial2);    
    rcCom.setup(Serial1);
    engineFR.setup(ENGINE_FR);
    engineFL.setup(ENGINE_FL);
    engineRR.setup(ENGINE_RR);
    engineRL.setup(ENGINE_RL);
    myMPU.setup();
}

uint16_t hz = 0;
unsigned long lastHzPrint = 0;
void loop(){
    unsigned long now = millis();
    if(now - lastHzPrint > 1000){
        lastHzPrint = now;
        Serial.print("HZ = ");
        Serial.println(hz);
        hz = 0;
    }
    hz++;
    telemetryCom.loop();
    rcCom.loop();    
    engineFR.setPower(rcCom.throttle);
    engineFL.setPower(rcCom.throttle);
    engineRR.setPower(rcCom.throttle);
    engineRL.setPower(rcCom.throttle);
    myMPU.loop();
    //Serial.println((int)((rcCom.throttle - 1000) / 10));
    //Serial.print("pitch =");
    //Serial.print(quadcopterPosition.ypr[0]); 
    /*Serial.print("pitch =");
    Serial.print(rcCom.pitch);    
    Serial.print(",roll =");
    Serial.print(rcCom.roll);    
    Serial.print(",yaw =");
    Serial.print(rcCom.yaw);
    Serial.print(",throttle =");
    Serial.print(rcCom.throttle);
    Serial.print(",p1 =");
    Serial.print(rcCom.p1);
    Serial.print(",p2 =");
    Serial.print(rcCom.p2);
    Serial.print(",intA =");
    Serial.print(rcCom.intA);
    Serial.print(",intB =");
    Serial.print(rcCom.intB);
    Serial.print(",intC =");
    Serial.print(rcCom.intC);
    Serial.print(",intD =");
    Serial.println(rcCom.intD);*/
}