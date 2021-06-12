#include "setup_board.h"

#include "config.h"
#include "led.h"

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void led_t::setup() {
    console->printf("A LED on pin: %d\n", HAL_GPIO_A_LED_PIN);
}

void led_t::update(int gyros_calibrated, const AP_GPS &gps) {
    if ( gyros_calibrated < 2 ) {
        blink_rate = 50;
    } else if ( gps.status() < AP_GPS::GPS_Status::GPS_OK_FIX_3D ) {
        blink_rate = 200;
    } else {
        blink_rate = 800;
    }
    if ( AP_HAL::millis() - blinkTimer >= blink_rate ) {
        blinkTimer = AP_HAL::millis();
        blink_state = !blink_state;
        hal.gpio->toggle(HAL_GPIO_A_LED_PIN);
        // console->printf("LED: %d\n", blink_state);
    }
}
        
// global shared instance
led_t led;
