#include <Arduino.h>
#include "ultrasonic_sense.h"

#define SOUND_SPEED 0.034

int Ultrasonic_Sensor::setup(const int ECHO_PIN, const int TRIG_PIN){
    this->echo_pin = ECHO_PIN;
    this->trig_pin = TRIG_PIN;

    Serial.begin(9600);
    pinMode(this->echo_pin, INPUT);
    pinMode(this->trig_pin, OUTPUT);

    return 1;
}

int Ultrasonic_Sensor::get_reading(){

    digitalWrite(this->trig_pin, LOW);
    delayMicroseconds(2);

    // Setting trig to high for 10 us
    digitalWrite(this->trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(this->trig_pin, LOW);

    // Read amt of time echo pin is high
    double t = pulseIn(this->echo_pin, HIGH);
    // double dist = (SOUND_SPEED * t)/2;
    double dist = (t / 2) * 0.0343;
    return dist;
}
