#include <AP_HAL/AP_HAL.h>

#include "setup_board.h"

#include "led.h"

void led_t::setup() {
#if defined(HAL_HAVE_PIXRACER_LED)
    console->printf("Have Pixracer LED\n");
#endif
    gps_node = PropertyNode("/sensors/gps");
}

void led_t::update() {
    if ( AP_HAL::millis() - blinkTimer >= blink_rate ) {
        blinkTimer = AP_HAL::millis();
        blink_state = !blink_state;
        if ( blink_state and r > 0 ) {
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
        } else {
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
        }
        if ( blink_state and g > 0 ) {
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
        } else {
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
        }
        if ( blink_state and b > 0 ) {
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
        } else {
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
        }
        // console->printf("LED: %d\n", blink_state);
    }
}

void led_t::do_policy(int gyros_calibrated) {
    if ( gyros_calibrated < 2 ) {
        set_color(255, 255, 0); // orange
        set_blink_rate(50);
    } else if ( gps_node.getInt("status") < 3 ) {
        set_color(0, 0, 255);   // blue
        set_blink_rate(200);
    } else {
        set_color(255, 255, 255); //  white
        set_blink_rate(800);
    }
}
