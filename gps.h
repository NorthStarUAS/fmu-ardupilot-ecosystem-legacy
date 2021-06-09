// GPS wrapper class

#pragma once

#define ALLOW_DOUBLE_MATH_FUNCTIONS
#include <AP_HAL/AP_HAL.h>

#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif

#pragma push_macro("_GLIBCXX_USE_C99_STDIO")
#undef _GLIBCXX_USE_C99_STDIO
#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#pragma pop_macro("_GLIBCXX_USE_C99_STDIO")

#include <AP_GPS/AP_GPS.h>

class gps_t {
    
public:
    AP_GPS gps;
    
    uint32_t last_message_ms = 0;
    uint32_t gps_millis = 0;
    bool gps_acquired = false;
    uint32_t gps_settle_timer = 0;
    // ublox8_nav_pvt_t gps_data;
    double unix_sec;
    float magvar_rad;
    Eigen::Vector3f mag_ned;

    void setup();
    void update();
    bool settle();
    
private:
    void update_unix_sec();
    void update_magvar();
};

extern gps_t the_gps;
