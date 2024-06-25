#include "mpu6050/src/MPU6050_6Axis_MotionApps20.h"

struct IMU_Readings {
    float acc_x, acc_y, acc_z;
    float gyro_x, gyro_y, gyro_z;
};

class IMU {
    unsigned char i2c_addr;
    MPU6050 mpu;
    public:
        void setup(unsigned char i2c_addr);
        IMU_Readings get_reading();

};
