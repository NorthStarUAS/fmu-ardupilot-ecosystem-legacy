// This is the section of code that derives, computes, and estimates
// things that aren't directly sensed.

#include "state_mgr.h"

void state_mgr_t::init() {
    airdata.init();
    switches.init();
}

void state_mgr_t::update() {
    airdata.update();
    switches.update();
}
