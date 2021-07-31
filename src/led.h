// das blinken class

#pragma once

#include "props2.h"

class led_t {
    
private:
    uint32_t blinkTimer = 0;
    unsigned int blink_rate = 100;
    bool blink_state = false;
    uint8_t r = 0, g = 0, b = 0;
    PropertyNode gps_node;
    
public:
    void init();
    void do_policy(int gyros_calibrated);
    void update();

    inline void set_blink_rate(uint16_t rate_ms) {
        blink_rate = rate_ms;
    }
    
    inline void set_color(uint8_t red_val, uint8_t green_val, uint8_t blue_val) {
        r = red_val;
        b = blue_val;
        g = green_val;
    }
};
