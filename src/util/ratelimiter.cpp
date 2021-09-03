#include "ratelimiter.h"

RateLimiter::RateLimiter() {
    dt_millis = 1000;
}

RateLimiter::RateLimiter( float hz ) {
    dt_millis = 1000 / hz;
}

bool RateLimiter::update() {
    if ( timer == 0 ) {
        timer = AP_HAL::millis();
    }
    if ( AP_HAL::millis() - timer >= DT_MILLIS ) {
        timer += dt_millis;
        return true;
    } else {
        return false;
    }
}
