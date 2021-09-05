#include <AP_Math/AP_Math.h>
#include "ratelimiter.h"

RateLimiter::RateLimiter() {
    dt_millis = 1000;
}

RateLimiter::RateLimiter( float hz ) {
    dt_millis = 1000.0 / hz;
}

bool RateLimiter::update() {
    if ( timer == 0 ) {
        timer = AP_HAL::millis() + (get_random16() % dt_millis);
    }
    if ( AP_HAL::millis() >= timer + dt_millis ) {
        if ( AP_HAL::millis() > timer + dt_millis ) {
            misses++;                 // oops
            timer = AP_HAL::millis(); // catchup
        } else {
            timer += dt_millis;       // all good
        }
        return true;
    } else {
        return false;
    }
}
