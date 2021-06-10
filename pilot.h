#pragma once

#include "aura4_messages.h"

const uint8_t MAX_RC_CHANNELS = 16;

class pilot_t {
private:
    // define if a channel is symmetrical or not (i.e. mapped to [0,1] for
    // throttle, flaps, spoilers; [-1,1] for aileron, elevator, rudder
    static const uint16_t pwm_symmetrical = (1 << 0) | (1 << 1) | (1 << 2);
    
    float pwm2norm(uint16_t pwm_val, uint8_t i);

    uint32_t last_input = 0;

public:
    uint16_t pwm_inputs[MAX_RC_CHANNELS]; // AP delivers pwm units
    float manual_inputs[MAX_RC_CHANNELS]; // normalized
    float ap_inputs[MAX_RC_CHANNELS];     // normalized
    bool failsafe = true;
    
    void setup();
    void update();
    
    void update_ap( message::command_inceptors_t *inceptors );

    // convenience
    inline bool ap_enabled() { return manual_inputs[0] >= 0.0; }
    inline bool throttle_safety() { return manual_inputs[1] < 0.0; }
    inline float get_aileron() {
        if ( ap_enabled() ) {
            return ap_inputs[3];
        } else {
            return manual_inputs[3];
        }
    }
    inline float get_elevator() {
        if ( ap_enabled() ) {
            return ap_inputs[4];
        } else {
            return manual_inputs[4];
        }
    }
    inline float get_throttle() {
        if ( ap_enabled() ) {
            return ap_inputs[2];
        } else {
            return manual_inputs[2];
        }
    }
    inline float get_rudder() {
        if ( ap_enabled() ) {
            return ap_inputs[5];
        } else {
            return manual_inputs[5];
        }
    }
    inline float get_flap() {
        if ( ap_enabled() ) {
            return ap_inputs[6];
        } else {
            return manual_inputs[6];
        }
    }
    inline float get_gear() {
        if ( ap_enabled() ) {
            return ap_inputs[7];
        } else {
            return manual_inputs[7];
        }
    }
    inline float get_ch7() {
        return manual_inputs[8];
    }
    inline float get_ch8() {
        return manual_inputs[9];
    }
};

extern pilot_t pilot;
