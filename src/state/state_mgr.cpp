// This is the section of code that derives, computes, and estimates
// things that aren't directly sensed.

#include "state_mgr.h"

void state_mgr_t::init() {
    switches.init();
    airdata.init();
    ground.init();
}

void state_mgr_t::update(float dt) {
    switches.update();
    airdata.update();
    ground.update(dt);
}
