#include "setup_board.h"

#include "config.h"
#include "led.h"

void led_t::defaults_goldy3() {
    config.board_cfg.led_pin = 0;
}

void led_t::defaults_aura3() {
    config.board_cfg.led_pin = 13;
}

void led_t::setup() {
    if ( config.board_cfg.led_pin > 0 ) {
        //pinMode(config.board_cfg.led_pin, OUTPUT);
        //digitalWrite(config.board_cfg.led_pin, HIGH);
        console->printf("LED on pin: %d\n", config.board_cfg.led_pin);
    } else {
        console->printf("No LED defined.\n");
    }
}

void led_t::update(int gyros_calibrated, int gps_fix) {
    if ( config.board_cfg.led_pin > 0 ) {

        if ( gyros_calibrated < 2 ) {
            blink_rate = 50;
        } else if ( gps_fix < 3 ) {
            blink_rate = 200;
        } else {
            blink_rate = 800;
        }
        if ( blinkTimer >= blink_rate ) {
            blinkTimer = 0;
            blink_state = !blink_state;
            // digitalWrite(config.board_cfg.led_pin, blink_state);
            console->printf("LED: %d\n", blink_state);
        }
    }
}
        
// global shared instance
led_t led;
