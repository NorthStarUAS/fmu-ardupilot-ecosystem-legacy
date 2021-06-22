// ins wrapper class

#pragma once

#include "setup_board.h"
#include "props2.h"
#include "nav/nav_structs.h"

#if defined(AURA_ONBOARD_EKF)
#include "nav/ekf15.h"
#include "nav/ekf15_mag.h"
#endif

class nav_mgr_t {
    
private:
    bool ekf_inited = false;
    unsigned long int gps_last_millis = 0;
#if defined(AURA_ONBOARD_EKF)
    EKF15 ekf;
    EKF15_mag ekf_mag;
#endif
    PropertyNode config_ekf_node;
    PropertyNode imu_node;
    
public:
    NAVdata data;
    uint8_t status;             // 0 = uninitted, 1 = no gps, 2 = 0k
    void setup();
    void update();
    void reinit();              // request the filter reinit itself
};

extern nav_mgr_t nav_mgr;
