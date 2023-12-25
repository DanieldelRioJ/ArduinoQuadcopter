//GND - GND
//VCC - VCC
//SDA - Pin A4
//SCL - Pin A5
//INT - Pin 2

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"

#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high


#define INTERRUPT_PIN 2

#define LED_PIN 13
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]
VectorInt16 aa;         // [x, y, z]
VectorInt16 aaReal;     // [x, y, z]
VectorInt16 aaWorld;    // [x, y, z]
VectorFloat gravity;    // [x, y, z]
float ypr[3];           // [yaw, pitch, roll]

volatile bool mpuInterrupt = false;
void dmpDataReady() {
    mpuInterrupt = true;
}



struct EulerAngles {
    double roll, pitch, yaw;
};

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(230400); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

    // Iniciar MPU6050
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // Comprobar  conexion
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // Iniciar DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Valores de calibracion
    mpu.setXAccelOffset(-1862);
    mpu.setYAccelOffset(-4684);
    mpu.setZAccelOffset(485);
    mpu.setXGyroOffset(123);
    mpu.setYGyroOffset(-30);
    mpu.setZGyroOffset(52);
    

    // Activar DMP
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Activar interrupcion
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

// this implementation assumes normalized quaternion
// converts to Euler angles in 3-2-1 sequence
double* toEulerAngles(Quaternion q) {

    double* ypr;
    ypr = new double[3];

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    ypr[2] = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    ypr[1] = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    ypr[0] = atan2(siny_cosp, cosy_cosp);

    return ypr;
}

Quaternion toQuaternion(float roll, float pitch, float yaw) // roll (x), pitch (y), yaw (z), angles are in radians
{
    // Abbreviations for the various angular functions

    float cr = cos(roll * 0.5);
    float sr = sin(roll * 0.5);
    float cp = cos(pitch * 0.5);
    float sp = sin(pitch * 0.5);
    float cy = cos(yaw * 0.5);
    float sy = sin(yaw * 0.5);

    Quaternion quat;
    quat.w = cr * cp * cy + sr * sp * sy;
    quat.x = sr * cp * cy - cr * sp * sy;
    quat.y = cr * sp * cy + sr * cp * sy;
    quat.z = cr * cp * sy - sr * sp * cy;

    return q;
}

uint32_t hz = 0;
unsigned long last = millis();
unsigned long now = millis();

void loop() {
    // Si fallo al iniciar, parar programa
    if (!dmpReady) return;

    // Ejecutar mientras no hay interrupcion
    while (!mpuInterrupt && fifoCount < packetSize) {
        // AQUI EL RESTO DEL CODIGO DE TU PROGRRAMA
    }

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Obtener datos del FIFO
    fifoCount = mpu.getFIFOCount();

    // Controlar overflow
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
    } 
    else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

    // MMostrar Yaw, Pitch, Roll
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print("ypr\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.println(ypr[2] * 180/M_PI);

    //Quaternion qRef = toQuaternion(0,0,0);
    //double* error = toEulerAngles(q);

    //Serial.print(error[0]);
    //Serial.print(error[1]);
    //Serial.println(error[2]);

    now = millis();
    hz++;
    if(now - last > 1000){
      last = now;
      //Serial.println(hz);
      hz = 0;
    }
    
    // Mostrar aceleracion
    /*mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    Serial.print("areal\t");
    Serial.print(aaReal.x);
    Serial.print("\t");
    Serial.print(aaReal.y);
    Serial.print("\t");
    Serial.println(aaReal.z);*/
    }
}