// IMU wrapper class

#pragma once

#include <AP_Math/AP_Math.h>

class imu_raw_t {
    
public:

    uint32_t raw_millis;
    Vector3f accel;
    Vector3f gyro;
    Vector3f mag;
    
    void setup();
    void update();
};
