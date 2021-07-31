// IMU wrapper class

#pragma once

#include <AP_Math/AP_Math.h>

class imu_hal_t {
    
public:

    uint32_t raw_millis;
    Vector3f accel;
    Vector3f gyro;
    float temp_C;
    Vector3f mag;
    
    void init();
    void update();
};
