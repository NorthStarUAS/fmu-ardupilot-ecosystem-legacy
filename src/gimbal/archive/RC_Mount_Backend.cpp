#include "RC_Mount_Backend.h"
#if HAL_MOUNT_ENABLED
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

// set_angle_targets - sets angle targets in degrees
void RC_Mount_Backend::set_angle_targets(float roll, float tilt, float pan)
{
    // set angle targets
    _angle_ef_target_rad.x = radians(roll);
    _angle_ef_target_rad.y = radians(tilt);
    _angle_ef_target_rad.z = radians(pan);

    // set the mode to mavlink targeting
    // _frontend.set_mode(_instance, MAV_MOUNT_MODE_MAVLINK_TARGETING);
    mount_node.setString("mode", "MAVLINK_TARGETING");
}

// set_roi_target - sets target location that mount should attempt to point towards
void RC_Mount_Backend::set_roi_target(const struct Location &target_loc)
{
    // set the target gps location
    //_state._roi_target = target_loc;
    //_state._roi_target_set = true;
    // set the mode to GPS tracking mode
    //_frontend.set_mode(_instance, MAV_MOUNT_MODE_GPS_POINT);
    mount_node.setDouble("roi_lat_deg", target_loc.lat);
    mount_node.setDouble("roi_lon_deg", target_loc.lng);
    int32_t target_alt_cm;
    if ( target_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt_cm) ) {
        mount_node.setInt("roi_alt_cm", target_alt_cm);
    }
    mount_node.setBool("roi_target_set", true);    
    mount_node.setString("mode", "GPS_POINT");
}

void RC_Mount_Backend::control(int32_t pitch_or_lat, int32_t roll_or_lon, int32_t yaw_or_alt, MAV_MOUNT_MODE mount_mode)
{
    // interpret message fields based on mode
    switch (mount_mode) {
        case MAV_MOUNT_MODE_RETRACT:
            mount_node.setString("mode", "RETRACT");
            break;
        case MAV_MOUNT_MODE_NEUTRAL:
            // do nothing with request if mount is retracted or in neutral position
            mount_node.setString("mode", "NEUTRAL");
            break;

        // set earth frame target angles from mavlink message
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            mount_node.setString("mode", "MAVLINK_TARGETING");
            set_angle_targets(roll_or_lon*0.01f, pitch_or_lat*0.01f, yaw_or_alt*0.01f);
            break;

        // Load neutral position and start RC Roll,Pitch,Yaw control with stabilization
        case MAV_MOUNT_MODE_RC_TARGETING:
            // do nothing if pilot is controlling the roll, pitch and yaw
            mount_node.setString("mode", "RC_TARGETING");
            break;

        // set lat, lon, alt position targets from mavlink message

        case MAV_MOUNT_MODE_GPS_POINT: {
            mount_node.setString("mode", "GPS_POINT");
            const Location target_location{
                pitch_or_lat,
                roll_or_lon,
                yaw_or_alt,
                Location::AltFrame::ABOVE_HOME
            };
            set_roi_target(target_location);
            break;
        }

        case MAV_MOUNT_MODE_HOME_LOCATION: {
            mount_node.setString("mode", "HOME_LOCATION");
            // set the target gps location
            //_state._roi_target = AP::ahrs().get_home();
            //_state._roi_target_set = true;
            set_roi_target(AP::ahrs().get_home());
            break;
        }

        default:
            // do nothing
            break;
    }
}

void RC_Mount_Backend::rate_input_rad(float &out, const RC_Channel *chan, float min, float max)
{
    if ((chan == nullptr) || (chan->get_radio_in() == 0)) {
        return;
    }
    out += chan->norm_input_dz() * 0.0001f * mount_node.getDouble("joystick_speed");
    out = constrain_float(out, radians(min*0.01f), radians(max*0.01f));
}

// update_targets_from_rc - updates angle targets using input from receiver
void RC_Mount_Backend::update_targets_from_rc()
{
    const RC_Channel *roll_ch = rc().channel(mount_node.getInt("roll_rc_in"));
    const RC_Channel *tilt_ch = rc().channel(mount_node.getInt("tilt_rc_in"));
    const RC_Channel *pan_ch = rc().channel(mount_node.getInt("pan_rc_in"));

    // if joystick_speed is defined then pilot input defines a rate of change of the angle
    if ( mount_node.getDouble("joystick_speed") ) {
        // allow pilot position input to come directly from an RC_Channel
        rate_input_rad(_angle_ef_target_rad.x,
                       roll_ch,
                       mount_node.getDouble("roll_angle_min"),
                       mount_node.getDouble("roll_angle_max"));
        rate_input_rad(_angle_ef_target_rad.y,
                       tilt_ch,
                       mount_node.getDouble("tilt_angle_min"),
                       mount_node.getDouble("tilt_angle_max"));
        rate_input_rad(_angle_ef_target_rad.z,
                       pan_ch,
                       mount_node.getDouble("pan_angle_min"),
                       mount_node.getDouble("pan_angle_max"));
    } else {
        // allow pilot rate input to come directly from an RC_Channel
        if ((roll_ch != nullptr) && (roll_ch->get_radio_in() != 0)) {
            _angle_ef_target_rad.x = angle_input_rad(roll_ch, mount_node.getDouble("roll_angle_min"), mount_node.getDouble("roll_angle_max"));
        }
        if ((tilt_ch != nullptr) && (tilt_ch->get_radio_in() != 0)) {
            _angle_ef_target_rad.y = angle_input_rad(tilt_ch, mount_node.getDouble("tilt_angle_min"), mount_node.getDouble("tilt_angle_max"));
        }
        if ((pan_ch != nullptr) && (pan_ch->get_radio_in() != 0)) {
            _angle_ef_target_rad.z = angle_input_rad(pan_ch, mount_node.getDouble("pan_angle_min"), mount_node.getDouble("pan_angle_max"));
        }
    }
}

// returns the angle (radians) that the RC_Channel input is receiving
float RC_Mount_Backend::angle_input_rad(const RC_Channel* rc, int16_t angle_min, int16_t angle_max)
{
    return radians(((rc->norm_input_ignore_trim() + 1.0f) * 0.5f * (angle_max - angle_min) + angle_min)*0.01f);
}

bool RC_Mount_Backend::calc_angle_to_roi_target(Vector3f& angles_to_target_rad,
                                                bool calc_tilt,
                                                bool calc_pan,
                                                bool relative_pan)
{
    if ( !mount_node.getBool("roi_target_set") ) {
        return false;
    }
    struct Location roi_target;
    roi_target.lat = mount_node.getDouble("target_lat_deg");
    roi_target.lng = mount_node.getDouble("target_lon_deg");
    roi_target.set_alt_cm(mount_node.getInt("target_alt_cm"),
                          Location::AltFrame::ABOVE_HOME);
    return calc_angle_to_location(roi_target, angles_to_target_rad, calc_tilt, calc_pan, relative_pan);
}

// calc_angle_to_location - calculates the earth-frame roll, tilt and pan angles (and radians) to point at the given target
bool RC_Mount_Backend::calc_angle_to_location(const struct Location &target, Vector3f& angles_to_target_rad, bool calc_tilt, bool calc_pan, bool relative_pan) const
{
    Location current_loc;
    if (!AP::ahrs().get_position(current_loc)) {
        return false;
    }
    const float GPS_vector_x = Location::diff_longitude(target.lng,current_loc.lng)*cosf(ToRad((current_loc.lat+target.lat)*0.00000005f))*0.01113195f;
    const float GPS_vector_y = (target.lat-current_loc.lat)*0.01113195f;
    int32_t target_alt_cm = 0;
    if (!target.get_alt_cm(Location::AltFrame::ABOVE_HOME, target_alt_cm)) {
        return false;
    }
    int32_t current_alt_cm = 0;
    if (!current_loc.get_alt_cm(Location::AltFrame::ABOVE_HOME, current_alt_cm)) {
        return false;
    }
    float GPS_vector_z = target_alt_cm - current_alt_cm;
    float target_distance = 100.0f*norm(GPS_vector_x, GPS_vector_y);      // Careful , centimeters here locally. Baro/alt is in cm, lat/lon is in meters.

    // initialise all angles to zero
    angles_to_target_rad.zero();

    // tilt calcs
    if (calc_tilt) {
        angles_to_target_rad.y = atan2f(GPS_vector_z, target_distance);
    }

    // pan calcs
    if (calc_pan) {
        // calc absolute heading and then onvert to vehicle relative yaw
        angles_to_target_rad.z = atan2f(GPS_vector_x, GPS_vector_y);
        if (relative_pan) {
            angles_to_target_rad.z = wrap_PI(angles_to_target_rad.z - AP::ahrs().yaw);
        }
    }
    return true;
}

#endif // HAL_MOUNT_ENABLED
