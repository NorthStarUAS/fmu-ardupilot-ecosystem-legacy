#include <AP_HAL/AP_HAL.h>
#include <AP_IOMCU/AP_IOMCU.h>

#include "setup_board.h"

#include "config.h"
#include "pilot.h"

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// 982 - 2006 (frsky) / 1496
static const uint16_t PWM_MIN = 982;
static const uint16_t PWM_MAX = 2006;
static const uint16_t PWM_CENTER = (PWM_MIN + PWM_MAX) / 2;
static const uint16_t PWM_HALF_RANGE = PWM_MAX - PWM_CENTER;
static const uint16_t PWM_RANGE = PWM_MAX - PWM_MIN;

float pilot_t::pwm2norm(uint16_t pwm_val, uint8_t i) {
    float norm = 0.0;
    if ( pwm_symmetrical & (1<<i) ) {
        // i.e. aileron, rudder, elevator
        norm = (float)((int)pwm_val - PWM_CENTER) / PWM_HALF_RANGE;
    } else {
        // i.e. throttle, flaps, etc.
        norm = (float)((int)pwm_val - PWM_MIN) / PWM_RANGE;
    }
    return norm;
}

uint16_t pilot_t::norm2pwm(float norm_val, uint8_t i) {
    uint16_t output = PWM_CENTER;
    if ( pwm_symmetrical & (1<<i) ) {
        output = PWM_CENTER + (int)(PWM_HALF_RANGE * norm_val); // * config.pwm_cfg.act_gain[i]);
    } else {
        output = PWM_MIN + (int)(PWM_RANGE * norm_val); // * config.pwm_cfg.act_gain[i]);
    }
    if ( output < PWM_MIN ) {
        output = PWM_MIN;
    }
    if ( output > PWM_MAX ) {
        output = PWM_MAX;
    }
    return output;
}

void pilot_t::setup() {
    manual_inputs[0] = ap_inputs[0] = -1.0; // autopilot disabled (manual)
    manual_inputs[1] = ap_inputs[1] = -1.0; // throttle safety enabled
    for ( int i = 2; i < MAX_RCIN_CHANNELS; i++ ) {
        manual_inputs[i] = ap_inputs[i] = 0.0;
    }

    mixer.setup();
}

bool pilot_t::read() {
    uint8_t nchannels = 0;
    bool new_input = false;
    if ( hal.rcin->new_input() ) {
        new_input = true;
        last_input = AP_HAL::millis();
        failsafe = false;
        nchannels = hal.rcin->read(pwm_inputs, MAX_RCIN_CHANNELS);
        for ( uint8_t i = 0; i < nchannels; i++ ) {
            manual_inputs[i] = pwm2norm(pwm_inputs[i], i);
        }
        // console->printf("%d ", nchannels);
        // for ( uint8_t i = 0; i < 8; i++ ) {
        //     console->printf("%.2f ", manual_inputs[i]);
        // }
        // console->printf("\n");
    } else if ( !failsafe and AP_HAL::millis() - last_input > 500 ) {
        failsafe = true;
        // console->printf("failsafe!\n");
    }
    return new_input;
}

void pilot_t::write() {
    // available inputs have been parsed/sorted so do the mixing right
    // before outputing the effector commands.
    mixer.update();
    for ( uint8_t i = 0; i < MAX_RCOUT_CHANNELS; i++ ) {
        float norm_val = mixer.outputs[i] * config.pwm_cfg.act_gain[i];
        uint16_t pwm_val = norm2pwm(norm_val, i);
        hal.rcout->write(i, pwm_val);
    }
}

void pilot_t::update_ap( message::command_inceptors_t *inceptors ) {
    // ap_inputs uses the same channel mapping as manual_inputs, so map
    // ap_tmp values to their correct places
    ap_inputs[0] = manual_inputs[0];      // auto/manual switch
    ap_inputs[1] = manual_inputs[1];      // throttle enable
    ap_inputs[2] = inceptors->channel[0]; // throttle
    ap_inputs[3] = inceptors->channel[1]; // aileron
    ap_inputs[4] = inceptors->channel[2]; // elevator
    ap_inputs[5] = inceptors->channel[3]; // rudder
    ap_inputs[6] = inceptors->channel[4]; // flap
    ap_inputs[7] = inceptors->channel[5]; // gear
}

// global shared instance
pilot_t pilot;
