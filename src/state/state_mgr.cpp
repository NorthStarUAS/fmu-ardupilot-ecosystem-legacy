// This is the section of code that derives, computes, and estimates
// things that aren't directly sensed.

#include "state_mgr.h"

void state_mgr_t::init() {
    airborne.init();
    switches.init();
}

void state_mgr_t::update() {
    airborne.update();
    switches.update();
}
