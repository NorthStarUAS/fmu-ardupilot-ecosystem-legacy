// IMU wrapper class

#pragma once

#include <AP_Math/AP_Math.h>

class imu_hal_t {
    
public:

    uint32_t raw_millis;
    Vector3f accel;
    Vector3f gyro;
    float tempC;
    Vector3f mag;
    
    void setup();
    void update();
};
