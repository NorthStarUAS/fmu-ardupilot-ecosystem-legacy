#include "RC_Mount_SToRM32.h"
#if HAL_MOUNT_ENABLED
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

RC_Mount_SToRM32::RC_Mount_SToRM32(/*RC_Mount &frontend, RC_Mount::mount_state &state, uint8_t instance*/) :
    RC_Mount_Backend(/*frontend, state, instance*/),
    _chan(MAVLINK_COMM_0)
{}

// update mount position - should be called periodically
void RC_Mount_SToRM32::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    // update based on mount mode
    string mode = mount_node.getString("mode");
    if ( mode == "MODE_RETRACT" ) {
        // move mount to a "retracted" position.  To-Do: remove
        // support and replace with a relaxed mode?
        _angle_ef_target_rad.x = ToRad(mount_node.getDouble("retract_roll_deg"));
        _angle_ef_target_rad.y = ToRad(mount_node.getDouble("retract_pitch_deg"));
        _angle_ef_target_rad.z = ToRad(mount_node.getDouble("retract_yaw_deg"));
    } else if ( mode == "NEUTRAL" ) {
        // move mount to a neutral position, typically pointing forward
        _angle_ef_target_rad.x = ToRad(mount_node.getDouble("neutral_roll_deg"));
        _angle_ef_target_rad.y = ToRad(mount_node.getDouble("neutral_pitch_deg"));
        _angle_ef_target_rad.z = ToRad(mount_node.getDouble("neutral_yaw_deg"));
    } else if ( mode == "MAVLINK_TARGETING" ) {
        // point to the angles given by a mavlink message ...
        // do nothing because earth-frame angle targets
        // (i.e. _angle_ef_target_rad) should have already been set by
        // a MOUNT_CONTROL message from GCS
        resend_now = true;
    } else if ( mode == "RC_TARGETING" ) {
        // RC radio manual angle control, but with stabilization from the AHRS
        // update targets using pilot's rc inputs
        update_targets_from_rc();
        resend_now = true;
    } else if ( mode == "GPS_POINT" ) {
        // point mount to a GPS point given by the mission planner
        if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true)) {
            resend_now = true;
        }
    } else if ( mode == "HOME_LOCATION" ) {
        // constantly update the home location:
        if ( AP::ahrs().home_is_set() ) {
            set_roi_target(AP::ahrs().get_home());
            if (calc_angle_to_roi_target(_angle_ef_target_rad, true, true)) {
                resend_now = true;
            }
        }
    }

    // resend target angles at least once per second
    if (resend_now || ((AP_HAL::millis() - _last_send) > AP_MOUNT_STORM32_RESEND_MS)) {
        send_do_mount_control(ToDeg(_angle_ef_target_rad.y), ToDeg(_angle_ef_target_rad.x), ToDeg(_angle_ef_target_rad.z), MAV_MOUNT_MODE_MAVLINK_TARGETING);
    }
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool RC_Mount_SToRM32::has_pan_control() const
{
    // we do not have yaw control
    return false;
}

// set_mode - sets mount's mode
void RC_Mount_SToRM32::set_mode(enum MAV_MOUNT_MODE mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // record the mode change
    switch (mode) {
        case MAV_MOUNT_MODE_RETRACT:
            mount_node.setString("mode", "RETRACT");
            break;
        case MAV_MOUNT_MODE_NEUTRAL:
            mount_node.setString("mode", "NEUTRAL");
            break;
        case MAV_MOUNT_MODE_MAVLINK_TARGETING:
            mount_node.setString("mode", "MAVLINK_TARGETING");
            break;
        case MAV_MOUNT_MODE_RC_TARGETING:
            mount_node.setString("mode", "RC_TARGETING");
            break;
        case MAV_MOUNT_MODE_GPS_POINT:
            mount_node.setString("mode", "GPS_POINT");
            break;
        case MAV_MOUNT_MODE_HOME_LOCATION:
            mount_node.setString("mode", "HOME_LOCATION");
            break;
        default:
            // do nothing
            break;
    }
}

// send_mount_status - called to allow mounts to send their status to GCS using the MOUNT_STATUS message
void RC_Mount_SToRM32::send_mount_status(mavlink_channel_t chan)
{
    // return target angles as gimbal's actual attitude.  To-Do: retrieve actual gimbal attitude and send these instead
    mavlink_msg_mount_status_send(chan, 0, 0, ToDeg(_angle_ef_target_rad.y)*100, ToDeg(_angle_ef_target_rad.x)*100, ToDeg(_angle_ef_target_rad.z)*100);
}

// search for gimbal in GCS_MAVLink routing table
void RC_Mount_SToRM32::find_gimbal()
{
    // return immediately if initialised
    if (_initialised) {
        return;
    }

    // return if search time has has passed
    if (AP_HAL::millis() > AP_MOUNT_STORM32_SEARCH_MS) {
        return;
    }

    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_GIMBAL, _sysid, _compid, _chan)) {
        _initialised = true;
    }
}

// send_do_mount_control - send a COMMAND_LONG containing a do_mount_control message
void RC_Mount_SToRM32::send_do_mount_control(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    // reverse pitch and yaw control
    pitch_deg = -pitch_deg;
    yaw_deg = -yaw_deg;

    // send command_long command containing a do_mount_control command
    mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_MOUNT_CONTROL,
                                  0,        // confirmation of zero means this is the first time this message has been sent
                                  pitch_deg,
                                  roll_deg,
                                  yaw_deg,
                                  0, 0, 0,  // param4 ~ param6 unused
                                  mount_mode);

    // store time of send
    _last_send = AP_HAL::millis();
}
#endif // HAL_MOUNT_ENABLED
