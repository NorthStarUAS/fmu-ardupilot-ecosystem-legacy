#pragma once

// max number of pwm output channels
const int MAX_PWM_CHANNELS = 8;

class pwm_t {
private:
    uint16_t gen_pwm_test_value();
    
public:
    uint16_t output_pwm[MAX_PWM_CHANNELS];
    void act_gain_defaults();
    void setup(int board);
    void update(uint8_t test_pwm_channel = -1);
    void norm2pwm( float *norm );
};

// a global instance is available for use
extern pwm_t pwm;
