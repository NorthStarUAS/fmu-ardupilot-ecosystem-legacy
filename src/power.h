#pragma once

#include <AP_HAL/AP_HAL.h>

#include "props2.h"

class power_t {
    
private:
    PropertyNode power_node;
    AP_HAL::AnalogSource *_volt_pin_analog_source;
    AP_HAL::AnalogSource *_curr_pin_analog_source;
    
public:
    float avionics_v = 0.0;
    float battery_volts = 0.0;
    float battery_amps = 0.0;
    
    void setup();
    void update();
};
