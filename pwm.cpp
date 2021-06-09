#include "setup_board.h"

#include "config.h"
#include "pwm.h"

// For a Futaba T6EX 2.4Ghz FASST system:
//   These number match a futaba T6EX FASST system output
//   Minimum position = 1107
//   Center position = 1520
//   Max position = 1933
static const int PWM_CENTER = 1520;
static const int PWM_HALF_RANGE = 413;
static const int PWM_QUARTER_RANGE = 206;
static const int PWM_RANGE = PWM_HALF_RANGE * 2;
static const int PWM_MIN = PWM_CENTER - PWM_HALF_RANGE;
static const int PWM_MAX = PWM_CENTER + PWM_HALF_RANGE;

static const uint8_t marmot1_pins[MAX_PWM_CHANNELS] = {21, 22, 23, 2, 3, 4, 5, 6};
static const uint8_t aura2_pins[MAX_PWM_CHANNELS] = {6, 5, 4, 3, 23, 22, 21, 20};
static uint8_t servoPins[MAX_PWM_CHANNELS];

// define if a channel is symmetrical or not (i.e. mapped to [0,1] for
// throttle, flaps, spoilers; [-1,1] for aileron, elevator, rudder
static bool pwm_symmetrical[MAX_PWM_CHANNELS] = {0, 1, 1, 1, 1, 0, 0, 0};

// This is the hardware PWM generation rate note the default is 50hz
// and this is the max we can drive analog servos.  Digital servos
// should be able to run at 200hz.  250hz is getting up close to the
// theoretical maximum of a 100% duty cycle.  Advantage for running
// this at 200+hz with digital servos is we should catch commanded
// position changes slightly faster for a slightly more responsive
// system (emphasis on slightly).  In practice, changing this to
// something higher than 50 hz has little practical effect and can
// often cause problems with ESC's that expect 50hz pwm signals.
static const int servoFreq_hz = 50; // servo pwm update rate

// reset actuator gains (reversing) to startup defaults
void pwm_t::act_gain_defaults() {
    for ( int i = 0; i < message::pwm_channels; i++ ) {
        config.pwm_cfg.act_gain[i] = 1.0;
    }
}

void pwm_t::setup(int board) {
    console->printf("PWM: ");
    if ( board == 0 ) {
        console->printf("Marmot v1 pin mapping.\n");
        for ( int i = 0; i < MAX_PWM_CHANNELS; i++ ) {
            servoPins[i] = marmot1_pins[i];
        }
    } else if ( board == 1 ) {
        console->printf("Aura v2 pin mapping.\n");
        for ( int i = 0; i < MAX_PWM_CHANNELS; i++ ) {
            servoPins[i] = aura2_pins[i];
        }
    } else {
        console->printf("No valid PWM pin mapping defined\n");
    }

    //analogWriteResolution(16);
    for ( int i = 0; i < MAX_PWM_CHANNELS; i++ ) {
        //analogWrite(servoPins[i], 0); // zero signal to avoid surprises
        //analogWriteFrequency(servoPins[i], servoFreq_hz);
    }
    
    // set default safe values for actuator outputs (should already at
    // a higher level, but this is important enough to do it again
    // just in case someone changed the higher level and messed up the
    // init order without realizing)
    // actuators.setup();
    // update();
}

// compute raw pwm values from normalized command values.  (handle
// actuator reversing here.)
void pwm_t::norm2pwm( float *norm ) {
    for ( int i = 0; i < MAX_PWM_CHANNELS; i++ ) {
        // convert to pulse length (special case ch6 when in flaperon mode)
        if ( pwm_symmetrical[i] /* FIXME: flaperon? */ ) {
            // i.e. aileron, rudder, elevator
            // Serial1.println(i);
            // Serial1.println(config_actuators.act_rev[i]);
            output_pwm[i] = PWM_CENTER + (int)(PWM_HALF_RANGE * norm[i] * config.pwm_cfg.act_gain[i]);
        } else {
            // i.e. throttle, flaps
            if ( config.pwm_cfg.act_gain[i] > 0.0 ) {
                output_pwm[i] = PWM_MIN + (int)(PWM_RANGE * norm[i] * config.pwm_cfg.act_gain[i]);
            } else {
                output_pwm[i] = PWM_MAX + (int)(PWM_RANGE * norm[i] * config.pwm_cfg.act_gain[i]);
            }
        }
        if ( output_pwm[i] < PWM_MIN ) {
            output_pwm[i] = PWM_MIN;
        }
        if ( output_pwm[i] > PWM_MAX ) {
            output_pwm[i] = PWM_MAX;
        }
    }
}


// write the raw actuator values to the RC system
void pwm_t::update(uint8_t test_pwm_channel) {
    // hook for testing servos
    if ( test_pwm_channel < MAX_PWM_CHANNELS ) {
        output_pwm[test_pwm_channel] = gen_pwm_test_value();
    }

    // sending servo pwm commands
    for ( uint8_t i = 0; i < MAX_PWM_CHANNELS; i++ ) {
        // analogWrite(servoPins[i], output_pwm[i] / ((1/((float) servoFreq_hz)) * 1000000.0f )*65535.0f);
    }
}

// test drive a servo channel (sine wave)
uint16_t pwm_t::gen_pwm_test_value() {
    return sin((float)AP_HAL::millis() / 500.0) * PWM_HALF_RANGE + PWM_CENTER;
}

// make a global instance
pwm_t pwm;
