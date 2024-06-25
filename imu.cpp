// #include "mpu6050.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <I2Cdev.h>
#include "imu.h"

#include "mpu6050/src/MPU6050_6Axis_MotionApps20.h"
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards

// Obtained register values from following:
// Documentation : https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

namespace nsMPU{
    MPU6050 mpu;
}

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU


void IMU::setup(unsigned char i2c_addr){
    // Wire.h library is used to communicate with I2C libraries
    // Sensor alignment is an important metric you want to account for

    // How to predict alignement erorr: The
    // devices combine a 3-axis gyroscope and a 3-axis accelerometer on the same silicon die together with an
    // onboard Digital Motion Processor™ (DMP™) capable of processing complex 9-axis sensor fusion algorithms
    // using the field-proven and proprietary MotionFusion™ engine.
  this->i2c_addr = i2c_addr;

  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock

  this->mpu.initialize();
  this->mpu.testConnection();
  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  Serial.println(F("Initializing DMP..."));
  uint8_t dev_status = this->mpu.dmpInitialize();

  nsMPU::mpu.setXGyroOffset(220);
      nsMPU::mpu.setYGyroOffset(76);
      nsMPU::mpu.setZGyroOffset(-85);
      nsMPU::mpu.setZAccelOffset(1788);

  if(dev_status == 0){
    mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
      this->mpu.setDMPEnabled(true);

      Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = this->mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

  } else {
    
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(dev_status);
        Serial.println(F(")"));
  }

}

IMU_Readings IMU::get_reading(){

    Serial.print("ORIENTATION : ");

    uint8_t fifo_buffer[64];
    Quaternion q; // [w, x, y, z]
    if (this->mpu.dmpGetCurrentFIFOPacket(fifo_buffer)) {
        this->mpu.dmpGetQuaternion(&q, fifo_buffer);

        float q0 = q.w;
        float q1 = q.x;
        float q2 = q.y;
        float q3 = q.z;

        float yr = -atan2(-2*q1*q2 + 2*q0*q3, q2*q2-q3*q3 - q1*q1 + q0*q0);
        float pr = asin(2*q2*q3 + 2*q0*q1);
        float rr = atan2(-2*q1*q3 + 2*q0*q2, q3*q3 - q2*q2 - q1*q1 + q0*q0);

        Serial.println("yaw"); Serial.print(yr);
        Serial.println("pitch"); Serial.print(pr);
        Serial.println("roll"); Serial.print(rr);
        IMU_Readings readings = {yr, pr, rr, 0,0,0};
        return readings;

    }
    
}
