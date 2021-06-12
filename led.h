// das blinken class

#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_GPS/AP_GPS.h>

class led_t {
private:
    uint32_t blinkTimer = 0;
    unsigned int blink_rate = 100;
    bool blink_state = true;
    
public:
    void setup();
    void update(int gyros_calibrated, const AP_GPS &gps);
};

extern led_t led;
