// Airdata class

#pragma once

#include <AP_Baro/AP_Baro.h>

#include "props2.h"

class airdata_t {

private:
    AP_Baro barometer;
    uint8_t counter = 0;
    uint32_t error_count = 0;
    bool pitot_found = false;
    PropertyNode airdata_node;
    
public:
    float baro_press = 0.0;
    float baro_temp = 0.0;
    float baro_hum = 0.0;
    float diffPress_pa = 0.0;
    float temp_C = 0.0;

    void setup();
    void update();
};
