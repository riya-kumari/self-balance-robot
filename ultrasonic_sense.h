
#include <Arduino.h>

class Ultrasonic_Sensor{
    int echo_pin;
    int trig_pin;

    public:
        int setup(const int ECHO_PIN, const int TRIG_PIN);
        int get_reading();
};
