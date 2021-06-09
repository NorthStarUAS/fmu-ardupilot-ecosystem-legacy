// das blinken class

#pragma once

class led_t {
private:
    uint32_t blinkTimer = 0;
    unsigned int blink_rate = 100;
    bool blink_state = true;
    
public:
    void defaults_goldy3();
    void defaults_aura3();
    void setup();
    void update(int gyros_calibrated, int gps_fix);
};

extern led_t led;
