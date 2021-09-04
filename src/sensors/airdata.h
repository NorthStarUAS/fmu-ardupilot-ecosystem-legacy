// Airdata class

#pragma once

#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Baro/AP_Baro.h>

#include "props2.h"

class airdata_t {

private:

    AP_Airspeed airspeed;
    AP_Baro barometer;
    uint8_t counter = 0;
    uint32_t error_count = 0;
    PropertyNode airdata_node;
    bool ready = false;
    
public:

    void init();
    void update();
};
