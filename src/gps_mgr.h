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
    uint32_t last_message_ms = 0;
    uint32_t gps_millis = 0;
    bool gps_acquired = false;
    uint32_t gps_settle_timer = 0;
    bool gps_settled = false;
    // ublox8_nav_pvt_t gps_data;
    uint64_t unix_usec = 0;
    uint64_t last_unix_usec = 0;
    float magvar_rad;
    Eigen::Vector3f mag_ned;

    void setup();
    void update();
    
private:
    AP_GPS gps;
    PropertyNode gps_node;
    void update_unix_sec();
    void update_magvar( time_t unix_sec );
};
