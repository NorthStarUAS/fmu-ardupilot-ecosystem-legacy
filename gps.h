// GPS wrapper class

#pragma once

// #include "sensors/UBLOX8/UBLOX8.h"

#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

class gps_t {
public:
    unsigned long gps_millis = 0;
    bool gps_acquired = false;
    // elapsedMillis gps_settle_timer = 0;
    // ublox8_nav_pvt_t gps_data;
    double unix_sec;
    float magvar_rad;
    // Vector3f mag_ned;

    void setup();
    void update();
    bool settle();
    
private:
    void update_unix_sec();
    void update_magvar();
};

extern gps_t gps;
