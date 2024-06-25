#include <Arduino.h>
#include "ultrasonic_sense.h"
#include "imu.h"

#include <Wire.h>

// define macros (constant)
#define ECHO_PIN 3
#define TRIG_PIN 2
#define I2C_ADDR 0x68

Ultrasonic_Sensor ultrasonic_sensor;
IMU imu;

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define OUTPUT_READABLE_ACCELGYRO


bool blinkState = false;

void setup() {

    Serial.begin(9600);
    ultrasonic_sensor.setup(ECHO_PIN, TRIG_PIN);
    imu.setup(I2C_ADDR);
}

void loop() {


    double dist = ultrasonic_sensor.get_reading();
    Serial.print("DISTANCE Reading: ");
    Serial.println(dist);
    delay(1000); // Delay for stability

    auto orientation = imu.get_reading();
    
    // Serial.println(orientation.acc_x);


}
