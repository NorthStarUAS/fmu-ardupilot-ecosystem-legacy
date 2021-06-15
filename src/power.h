#pragma once

#include <AP_HAL/AP_HAL.h>

class power_t {
    
private:
    //const float analogResolution = 65535.0f;
    //const float pwr_scale = 11.0f;
    //const float avionics_scale = 2.0f;
    //uint8_t avionics_pin;
    //uint8_t source_volt_pin;
    //uint8_t atto_volts_pin = 0 /*A2*/;
    //uint8_t atto_amps_pin = 0 /*A3*/;
    AP_HAL::AnalogSource *_volt_pin_analog_source;
    AP_HAL::AnalogSource *_curr_pin_analog_source;
    
public:
    float avionics_v = 0.0;
    float battery_volts = 0.0;
    float battery_amps = 0.0;
    
    void setup();
    void update();
};

extern power_t power;
