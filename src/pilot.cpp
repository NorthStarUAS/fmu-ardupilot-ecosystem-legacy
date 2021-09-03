#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h> // fixme: debugging

#include "setup_board.h"

#include "pilot.h"

// 982 - 2006 (frsky) / 1496
static const uint16_t PWM_MIN = 982;
static const uint16_t PWM_MAX = 2006;
static const uint16_t PWM_CENTER = (PWM_MIN + PWM_MAX) / 2;
static const uint16_t PWM_HALF_RANGE = PWM_MAX - PWM_CENTER;
static const uint16_t PWM_RANGE = PWM_MAX - PWM_MIN;

float pilot_t::rcin2norm(uint16_t pwm_val, uint8_t channel) {
    float norm = 0.0;
    if ( rcin_symmetrical & (1<<channel) ) {
        // i.e. aileron, rudder, elevator
        norm = (float)((int)pwm_val - PWM_CENTER) / PWM_HALF_RANGE;
    } else {
        // i.e. throttle, flaps, etc.
        norm = (float)((int)pwm_val - PWM_MIN) / PWM_RANGE;
    }
    return norm;
}

uint16_t pilot_t::norm2rcout(float norm_val, uint8_t channel) {
    uint16_t output = PWM_CENTER;
    if ( rcout_symmetrical & (1<<channel) ) {
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

void pilot_t::init() {
    config_eff_gains = PropertyNode("/config/pwm");
    effector_node = PropertyNode("/effectors");
    pilot_node = PropertyNode("/pilot");
    rcin_node = PropertyNode("/sensors/rc-input");
    
    manual_inputs[0] = ap_inputs[0] = -1.0; // autopilot disabled (manual)
    manual_inputs[1] = ap_inputs[1] = -1.0; // throttle safety enabled
    for ( int i = 2; i < MAX_RCIN_CHANNELS; i++ ) {
        manual_inputs[i] = ap_inputs[i] = 0.0;
    }

    // extend gain array with default value (1.0) if not provided in
    // config file
    uint8_t size = config_eff_gains.getLen("gains");
    for ( uint8_t i = size; i < MAX_RCOUT_CHANNELS; i++ ) {
        config_eff_gains.setDouble("gains", 1.0, i);
    }
    
    // enable channels
    for ( uint8_t i = 0; i < 6 /*MAX_RCOUT_CHANNELS*/; i++ ) {
        hal.rcout->enable_ch(i);
    }
    // for ( uint8_t i = MAX_RCOUT_CHANNELS; i < 14; i++ ) {
    //     hal.rcout->enable_ch(i);
    // }
    hal.rcout->force_safety_off();
    hal.util->set_soft_armed(true);
    
    mixer.init();
    switches.init();
}

bool pilot_t::read() {
    uint8_t nchannels = 0;
    bool new_input = false;
    if ( hal.rcin->new_input() ) {
        new_input = true;
        changed = true;
        last_input = AP_HAL::millis();
        pilot_node.setUInt("millis", last_input);
        nchannels = hal.rcin->read(pwm_inputs, MAX_RCIN_CHANNELS);
        for ( uint8_t i = 0; i < nchannels; i++ ) {
            rcin_node.setUInt("channel", pwm_inputs[i], i);
            manual_inputs[i] = rcin2norm(pwm_inputs[i], i);
            pilot_node.setDouble("channel", manual_inputs[i], i);
        }
        
        // logical values
        pilot_node.setBool("failsafe", false); // good
        pilot_node.setBool("ap_enabled", ap_enabled());
        pilot_node.setBool("throttle_safety", throttle_safety());
        pilot_node.setDouble("aileron", get_aileron());
        pilot_node.setDouble("elevator", get_elevator());
        pilot_node.setDouble("throttle", get_throttle());
        pilot_node.setDouble("rudder", get_rudder());
        pilot_node.setDouble("flaps", get_flap());
        pilot_node.setDouble("gear", get_gear());
        pilot_node.setDouble("aux1", get_aux1());
        pilot_node.setDouble("aux2", get_aux2());
        // console->printf("%d ", nchannels);
        // for ( uint8_t i = 0; i < 8; i++ ) {
        //     console->printf("%.2f ", manual_inputs[i]);
        // }
        // console->printf("\n");
    } else if ( AP_HAL::millis() - last_input > 500 and !pilot_node.getBool("failsafe") ) {
        pilot_node.setBool("failsafe", true); // bad
        // console->printf("failsafe!\n");
    }
    return new_input;
}

void pilot_t::write() {
    // available inputs have been parsed/sorted so do the mixing right
    // before outputing the effector commands.
    mixer.update();
    switches.update();

    /*console->printf("safety: %d armed: %d\n",
                    hal.util->safety_switch_state(),
                    hal.util->get_soft_armed());*/

    for ( uint8_t i = 0; i < MAX_RCOUT_CHANNELS; i++ ) {
        // float norm_val = mixer.outputs[i] * config.pwm_cfg.act_gain[i];
        float norm_val = effector_node.getDouble("channel", i)
            * config_eff_gains.getDouble("gains", i);
        uint16_t pwm_val = norm2rcout(norm_val, i);
        // console->printf("%d ", pwm_val);
        hal.rcout->write(i, pwm_val);
    }
    // console->printf("\n");
    // pwm_test = 1000 + (AP_HAL::millis() % 5000) / 5;
    // for ( uint8_t i = MAX_RCOUT_CHANNELS; i < 14; i++ ) {
    //     hal.rcout->write(i, pwm_test);
    // }
    changed = false;
}

void pilot_t::update_ap( rcfmu_message::command_inceptors_t *inceptors ) {
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
    for ( int i = 0; i < MAX_RCIN_CHANNELS; i++ ) {
        pilot_node.setDouble("auto", ap_inputs[i], i);
    }
    changed = true;
}

// global shared instance
pilot_t pilot;
