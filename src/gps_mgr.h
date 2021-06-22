// GPS wrapper class

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>

#include "setup_board.h"

#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "props2.h"

class gps_mgr_t {
    
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
    PropertyNode gps_node;
    void update_unix_sec();
    void update_magvar();
};

extern gps_mgr_t gps_mgr;
