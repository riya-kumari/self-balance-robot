
#include <Arduino.h>

class Motor{
    int pin1;
    int pin2;

    public:
        int setup(const int pin1, const int pin2);
        int move(double direction, );
};
