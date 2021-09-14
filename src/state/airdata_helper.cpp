// determine if aircraft if airborne or on the ground and time the
// airborne seconds

#include "airdata_helper.h"

void airdata_helper_t::init() {
    airdata_node = PropertyNode("/sensors/airdata/0");
    pos_node = PropertyNode("/position");
    vel_node = PropertyNode("/velocity");
    
    float cruise_kts = PropertyNode("/config/specs").getDouble("cruise_kt");
    if ( cruise_kts > 1.0 ) {
        up_kts = cruise_kts * 0.6;
        down_kts = cruise_kts * 0.4;
    }
    airdata_node.setBool("is_airborne", false);
}

void airdata_helper_t::update() {
    // determine if aircraft is flying or not
    if ( !is_airborne and pos_node.getDouble("altitude_agl_ft") >= up_ft and vel_node.getDouble("airspeed_kt") >= up_kts ) {
        // if all conditions over the threshold, we are airborne
        is_airborne = true;
        airdata_node.setBool("is_airborne", true);
        // fixme! comms.events.log("mission", "airborne");
    } else if ( is_airborne and pos_node.getDouble("altitude_agl_ft") <= down_ft and vel_node.getDouble("airspeed_kt") <= down_kts ) {
        // if all conditions under their threshold, we are on the ground
        is_airborne = false;
        airdata_node.setBool("is_airborne", false);
        // fixme! comms.events.log("mission", "on ground");
    }
    
    // compute total time aloft
    if ( is_airborne ) {
        flight_millis += AP_HAL::millis() - last_millis;
    }
    airdata_node.setDouble("flight_timer_millis", flight_millis);
    last_millis = AP_HAL::millis();
}
