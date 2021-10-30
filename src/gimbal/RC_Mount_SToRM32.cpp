#include "RC_Mount_SToRM32.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#include <GCS_MAVLink/include/mavlink/v2.0/ardupilotmega/mavlink_msg_mount_control.h>

extern const AP_HAL::HAL& hal;

RC_Mount_SToRM32::RC_Mount_SToRM32(/*RC_Mount &frontend, RC_Mount::mount_state &state, uint8_t instance*/) :
    RC_Mount_Backend(/*frontend, state, instance*/),
    _chan(MAVLINK_COMM_0)
{}

void RC_Mount_SToRM32::init()
{
    mount_node = PropertyNode("/gimbal");
    _port = hal.serial(1);           // telem 1
    _port->begin(115200);
}

void RC_Mount_SToRM32::read_messages() {
    while ( _port->available() >= 1 ) {
        uint8_t input = _port->read();
        mavlink_message_t in_msg;
        mavlink_status_t in_status;
        if ( mavlink_parse_char(MAVLINK_COMM_1, input, &in_msg, &in_status) ) {
            printf("Received message with ID %d, sequence: %d from component %d of sysid %d\n", in_msg.msgid, in_msg.seq, in_msg.compid, in_msg.sysid);
            if ( ! _found_gimbal and in_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                printf("The gimbal is talking to us!\n");
                _found_gimbal = true;
                _sysid = in_msg.sysid;
                _compid = in_msg.compid;
            } else if ( in_msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK ) {
                mavlink_command_ack_t ack_msg;
                mavlink_msg_command_ack_decode(&in_msg, &ack_msg);
                printf("Received message ack to command id = %d\n",
                       ack_msg.command);
                printf("  result = %d\n", ack_msg.result);
                // printf("  result_param2 = %d\n", ack_msg.result_param2);
            }
        }
    }
}

#define SYSID_ONBOARD 4
void RC_Mount_SToRM32::send_heartbeat() {
    if ( (AP_HAL::millis() - _last_heartbeat) > 10000 ) {
        set_gimbal_mode();
        printf("Sending HB to gimbal.\n");
        mavlink_heartbeat_t heartbeat;
        heartbeat.type 		= MAV_TYPE_ONBOARD_CONTROLLER;
        heartbeat.autopilot 	= MAV_AUTOPILOT_GENERIC;
        heartbeat.base_mode 	= 0;
        heartbeat.custom_mode 	= 0;
        heartbeat.system_status = MAV_STATE_ACTIVE;
        mavlink_message_t message;
        mavlink_msg_heartbeat_encode(SYSID_ONBOARD, MAV_COMP_ID_SYSTEM_CONTROL, &message, &heartbeat);
        uint8_t buf[300];
        uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &message);
        printf("  send buffer len = %d\n", buf_len);
        _port->write((uint8_t *)&buf, buf_len);
        _last_heartbeat = AP_HAL::millis();
    }
}

#define LOCK_MODE 0x01    // solid green
#define FOLLOW_MODE 0x02  // blinking green
void RC_Mount_SToRM32::set_gimbal_mode() {
    printf("set_gimbal_mode()\n");
    uint8_t buf[300];
    uint16_t buf_len = 0;
    if ( true ) {
        mavlink_command_long_t comm = { 0 };
        comm.target_system    	= _sysid;
        comm.target_component 	= MAV_COMP_ID_GIMBAL;
        comm.command            = MAV_CMD_USER_2;
        comm.param7             = LOCK_MODE;
        comm.confirmation     	= false;
        mavlink_message_t message;
        mavlink_msg_command_long_encode(_sysid, MAV_COMP_ID_SYSTEM_CONTROL, &message, &comm);
        buf_len = mavlink_msg_to_send_buffer(buf, &message);
        printf("  send buffer len = %d\n", buf_len);
    }
    if ( false ) {
        mavlink_message_t msg;
        uint16_t msg_len = mavlink_msg_command_long_pack(0,
                                                         0,
                                                         &msg,
                                                         0,
                                                         MAV_COMP_ID_GIMBAL,
                                                         MAV_CMD_USER_2,
                                                         0,        // confirmation of zero means this is the first time this message has been sent
                                                         0, // param 1-3 unused
                                                         0,
                                                         0,
                                                         0, 0, 0,  // param4 ~ param6 unused
                                                         FOLLOW_MODE);
        printf("mount control message len = %d\n", msg_len);
        buf_len = mavlink_msg_to_send_buffer(buf, &msg);
        printf("  send buffer len = %d\n", buf_len);
    }
    _port->write((uint8_t *)&buf, buf_len);
}

// update mount position - should be called periodically
void RC_Mount_SToRM32::update() {
    send_heartbeat();
    
    read_messages();

    // flag to trigger sending target angles to gimbal
    bool resend_now = false;

    // update based on mount mode
    string mode = mount_node.getString("mode");
    if ( mode == "" ) {
        mode = "GPS_POINT";
    }
    
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
        double sec = AP_HAL::millis() / 1000.0;
        _angle_ef_target_rad.x = 10*sin(sec*0.5);
        _angle_ef_target_rad.y = 10*sin(sec*0.2);
        _angle_ef_target_rad.z = 10*sin(sec*0.1);
        /*printf("Trying to send to: %.1f %.1f %.1f (deg)\n",
               _angle_ef_target_rad.y,
               _angle_ef_target_rad.x,
               _angle_ef_target_rad.z);*/
               
        send_do_mount_control(_angle_ef_target_rad.y, _angle_ef_target_rad.x, _angle_ef_target_rad.z, MAV_MOUNT_MODE_MAVLINK_TARGETING);
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
    if (! _found_gimbal) {
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

// send_do_mount_control - send a COMMAND_LONG containing a do_mount_control message
void RC_Mount_SToRM32::send_do_mount_control(float pitch_deg, float roll_deg, float yaw_deg, enum MAV_MOUNT_MODE mount_mode)
{
    return;
    
    // exit immediately if not initialised
    if ( !_found_gimbal ) {
        return;
    }

    // check we have space for the message
    if (false and /*disable check*/ !HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

    // reverse pitch and yaw control
    pitch_deg = -pitch_deg;
    yaw_deg = -yaw_deg;

    uint16_t msg_len = 0;
    mavlink_message_t msg;
    //msg_len = mavlink_msg_mount_control_pack(0, MAV_COMP_ID_GIMBAL, &msg, 0, 0, 10*100, 10*100, 10*100, 0);
    //printf("mount control message len = %d\n", msg_len);
    //_port->write((uint8_t *)&msg, msg_len);
    // _port->write("hello from pixhaw4\r\n");
    
    // send command_long command containing a do_mount_control command
    // mavlink_msg_command_long_send(_chan,
    //                               _sysid,
    //                               _compid,
    //                               MAV_CMD_DO_MOUNT_CONTROL,
    //                               0,        // confirmation of zero means this is the first time this message has been sent
    //                               pitch_deg,
    //                               roll_deg,
    //                               yaw_deg,
    //                               0, 0, 0,  // param4 ~ param6 unused
    //                               mount_mode);
    msg_len = mavlink_msg_command_long_pack(0,
                                  0,
                                  &msg,
                                  0,
                                  MAV_COMP_ID_GIMBAL,
                                  MAV_CMD_DO_MOUNT_CONTROL,
                                  0,        // confirmation of zero means this is the first time this message has been sent
                                  pitch_deg,
                                  roll_deg,
                                  yaw_deg,
                                  0, 0, 0,  // param4 ~ param6 unused
                                  mount_mode);
    printf("mount control message len = %d\n", msg_len);
    uint8_t buf[300];
    uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &msg);
    printf("  send buffer len = %d\n", buf_len);
    // for ( int i = 0; i < buf_len; i++ ) {
    //     if ( ((uint8_t *)(&msg))[i] != buf[i] ) {
    //         printf("    byte %d mismatch %d %d\n", i, ((uint8_t *)(&msg))[i], buf[i]);
    //     }
    // }
    // _port->write((uint8_t *)&buf, buf_len);
    

    // store time of send
    _last_send = AP_HAL::millis();
}
