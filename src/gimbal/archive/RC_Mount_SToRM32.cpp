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
    // write_buf = new ByteBuffer(300);
    mount_node = PropertyNode("/gimbal");
    imu_node = PropertyNode("/gimbal/imu");
    pilot_node = PropertyNode("/pilot");
    _port = hal.serial(1);           // telem 1
    _port->begin(115200);
    hal.scheduler->delay(100);
}

void RC_Mount_SToRM32::read_messages() {
    while ( _port->available() >= 1 ) {
        uint8_t input = _port->read();
        mavlink_message_t in_msg;
        mavlink_status_t in_status;
        if ( mavlink_parse_char(MAVLINK_COMM_1, input, &in_msg, &in_status) ) {
            // printf("Received message with ID %d, sequence: %d from component %d of sysid %d\n", in_msg.msgid, in_msg.seq, in_msg.compid, in_msg.sysid);
            if ( in_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
                if ( !_found_gimbal ) {
                    printf("The gimbal is talking to us!\n");
                    _found_gimbal = true;
                    _sysid = in_msg.sysid;
                    _compid = in_msg.compid;
                }
            } else if ( in_msg.msgid == MAVLINK_MSG_ID_COMMAND_ACK ) {
                mavlink_command_ack_t ack_msg;
                mavlink_msg_command_ack_decode(&in_msg, &ack_msg);
                /*
                printf("Received message ack to command id = %d\n",
                       ack_msg.command);
                printf("  result = %d\n", ack_msg.result);
                printf("  result_param2 = %d\n", ack_msg.result_param2);
                */
            } else if ( in_msg.msgid == MAVLINK_MSG_ID_SYS_STATUS ) {
                mavlink_sys_status_t msg;
                mavlink_msg_sys_status_decode(&in_msg, &msg);
                //printf("Gimbal sys status:\n");
                //printf("  voltage_battery: %d V\n", msg.voltage_battery);
                mount_node.setUInt("input_volts", msg.voltage_battery);
            } else if ( in_msg.msgid == MAVLINK_MSG_ID_MOUNT_STATUS ) {
                mavlink_mount_status_t msg;
                mavlink_msg_mount_status_decode(&in_msg, &msg);
                /*printf("Gimbal mount status:\n");
                printf("  angles: %d %d %d\n", msg.pointing_a, msg.pointing_b, msg.pointing_c);
                printf("  target: %d %d\n", msg.target_system, msg.target_component);
                mount_node.setUInt("battery_volts", msg.voltage_battery);*/
            } else if ( in_msg.msgid == MAVLINK_MSG_ID_MOUNT_ORIENTATION ) {
                mavlink_mount_orientation_t msg;
                mavlink_msg_mount_orientation_decode(&in_msg, &msg);
                // printf("Gimbal mount orientation:\n");
                // printf("  angles: %.1f %.1f %.1f %.1f\n", msg.roll, msg.pitch, msg.yaw, msg.yaw_absolute);
                mount_node.setDouble("relative_roll_deg", msg.roll);
                mount_node.setDouble("relative_pitch_deg", msg.pitch);
                mount_node.setDouble("relative_yaw_deg", msg.yaw);
                mount_node.setDouble("absolute_yaw_deg", msg.yaw_absolute);
            } else if ( in_msg.msgid == MAVLINK_MSG_ID_RAW_IMU ) {
                _receiving_imu = true;
                mavlink_raw_imu_t msg;
                mavlink_msg_raw_imu_decode(&in_msg, &msg);
                /*
                printf("Gimbal raw imu:\n");
                //printf("  time:  %d\n", msg.time_usec); // zero
                printf("  accel: %d %d %d\n", msg.xacc, msg.yacc, msg.zacc);
                printf("  gyro:  %d %d %d\n", msg.xgyro, msg.ygyro, msg.zgyro);
                printf("  mags:  %d %d %d\n", msg.xmag, msg.ymag, msg.zmag);
                //printf("  temp:  %d\n", msg.temperature); // bogus?
                */
                imu_node.setDouble("ax_mps2", -msg.yacc / 1000.0);
                imu_node.setDouble("ay_mps2", msg.xacc / 1000.0);
                imu_node.setDouble("az_mps2", msg.zacc / 1000.0);
                imu_node.setDouble("p_rps", msg.ygyro / 1000.0);
                imu_node.setDouble("q_rps", -msg.xgyro / 1000.0);
                imu_node.setDouble("r_rps", msg.zgyro / 1000.0);
		imu_node.setUInt("millis", AP_HAL::millis());
		imu_node.setDouble("timestamp", AP_HAL::millis() / 1000.0);
            } else {
                printf("Recieved unknown msgid: %d\n", in_msg.msgid);
            }
        }
    }
}

#define SYSID_ONBOARD 4
void RC_Mount_SToRM32::send_heartbeat() {
    if ( (AP_HAL::millis() - _last_heartbeat) > 1000 ) {
        if ( ! _receiving_imu ) {
            set_imu_rate();
        }
        set_gimbal_mode();
        set_mount_configure();
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

void RC_Mount_SToRM32::set_imu_rate() {
    printf("Sending imu rate to gimbal.\n");
    mavlink_param_set_t param_set = {0};
    param_set.param_value = 50;       // Onboard parameter value
    param_set.target_system	= _sysid; // System ID
    param_set.target_component = MAV_COMP_ID_GIMBAL; // Component ID
    mav_array_memcpy(param_set.param_id, "IMU_RATE", sizeof("IMU_RATE"));
    param_set.param_type		= MAVLINK_TYPE_UINT16_T;
    mavlink_message_t message;
    mavlink_msg_param_set_encode(SYSID_ONBOARD, MAV_COMP_ID_SYSTEM_CONTROL, &message, &param_set);
    uint8_t buf[300];
    uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &message);
    printf("  send buffer len = %d\n", buf_len);
    _port->write((uint8_t *)&buf, buf_len);
}

void RC_Mount_SToRM32::set_gimbal_mode() {
    printf("set_gimbal_mode()\n");
    //uint8_t buf[300];
    //uint16_t buf_len = 0;
    mavlink_command_long_t comm = { 0 };
    comm.target_system    	= _sysid;
    comm.target_component 	= MAV_COMP_ID_GIMBAL;
    comm.command            = MAV_CMD_USER_2;
    comm.param7             = 0x01; // 0x00 motors off, 0x01 lock mode (blinking green), 0x02 follow mode (solid green)
    comm.confirmation     	= false;
    mavlink_message_t message;
    mavlink_msg_command_long_encode(_sysid, MAV_COMP_ID_SYSTEM_CONTROL, &message, &comm);
    //buf_len = mavlink_msg_to_send_buffer(buf, &message);
    //printf("  send buffer len = %d\n", buf_len);
    //_port->write((uint8_t *)&buf, buf_len);
}

void RC_Mount_SToRM32::set_mount_configure() {
    printf("set_mount_configure()\n");
    //uint8_t buf[300];
    //uint16_t buf_len = 0;
    mavlink_command_long_t comm = { 0 };
    comm.target_system    	= _sysid;
    comm.target_component 	= MAV_COMP_ID_GIMBAL;
    comm.command            = MAV_CMD_DO_MOUNT_CONFIGURE;
    comm.param1             = MAV_MOUNT_MODE_MAVLINK_TARGETING; // operation mode
    comm.param2             = 1;   // stabalize roll
    comm.param3             = 1;   // stabalize pitch
    comm.param4             = 1;   // stabalize yaw
    comm.param5             = 0;   // roll body angles (encoder)
    comm.param6             = 0;   // pitch body angles (encoder)
    comm.param7             = 0;   // yaw body angles (encoder)
    comm.confirmation       = false;
    mavlink_message_t message;
    mavlink_msg_command_long_encode(_sysid, MAV_COMP_ID_SYSTEM_CONTROL, &message, &comm);
    //buf_len = mavlink_msg_to_send_buffer(buf, &message);
    //printf("  send buffer len = %d\n", buf_len);
    //_port->write((uint8_t *)&buf, buf_len);
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
    if (resend_now || ((AP_HAL::millis() - _last_send) > 100)) {
        //double sec = AP_HAL::millis() / 1000.0;
        double target_roll = 30 * pilot_node.getDouble("channel", 0);
        double target_pitch = 30 * pilot_node.getDouble("channel", 1);
        double target_yaw = 30 * pilot_node.getDouble("channel", 3);
        double curr_roll = mount_node.getDouble("relative_roll_deg");
        double curr_pitch = mount_node.getDouble("relative_pitch_deg");
        double curr_yaw = mount_node.getDouble("relative_yaw_deg");
        double cmd_roll = curr_roll - target_roll;
        double cmd_pitch = curr_pitch - target_pitch;
        double cmd_yaw = curr_yaw - target_yaw;
        if ( cmd_roll < -180 ) { cmd_roll += 360; }
        if ( cmd_roll > 180 ) { cmd_roll -= 360; }
        if ( cmd_pitch < -180 ) { cmd_pitch += 360; }
        if ( cmd_pitch > 180 ) { cmd_pitch -= 360; }
        if ( cmd_yaw < -180 ) { cmd_yaw += 360; }
        if ( cmd_yaw > 180 ) { cmd_yaw -= 360; }
        /*printf("Trying to send to: %.1f %.1f %.1f (deg)\n",
               _angle_ef_target_rad.y,
               _angle_ef_target_rad.x,
               _angle_ef_target_rad.z);*/
	// don't send micro adjustments
	float thresh = 0.5;	// deg
	if ( fabs(cmd_roll) > thresh or fabs(cmd_pitch) > thresh or fabs(cmd_yaw) > thresh ) {
	  send_do_mount_control(cmd_pitch, -cmd_roll, cmd_yaw, MAV_MOUNT_MODE_MAVLINK_TARGETING);
	}
    }

    /*
    uint8_t buf[300];
    uint16_t avail = write_buf->available();
    uint16_t got = write_buf->peekbytes((uint8_t *)buf, avail);
    uint16_t result = _port->write((uint8_t *)&buf, got);
    if ( result > 0 ) {
        printf("  bytes written = %d of %d\n", result, got);
        write_buf->advance(result);
    }
    */
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

    // uint16_t msg_len = 0;
    // mavlink_message_t msg;
    
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
    
    // msg_len = mavlink_msg_command_long_pack(0,
    //                               0,
    //                               &msg,
    //                               0,
    //                               MAV_COMP_ID_GIMBAL,
    //                               MAV_CMD_DO_MOUNT_CONTROL,
    //                               0,        // confirmation of zero means this is the first time this message has been sent
    //                               pitch_deg,
    //                               roll_deg,
    //                               yaw_deg,
    //                               0, 0, 0,  // param4 ~ param6 unused
    //                               mount_mode);
    // printf("mount control message len = %d\n", msg_len);
    // uint8_t buf[300];
    // uint16_t buf_len = mavlink_msg_to_send_buffer(buf, &msg);
    // printf("  send buffer len = %d\n", buf_len);
    
    printf("do_mount_control()\n");
    uint8_t buf[300];
    uint16_t buf_len = 0;
    mavlink_command_long_t comm = { 0 };
    comm.target_system    	= _sysid;
    comm.target_component 	= MAV_COMP_ID_GIMBAL;
    comm.command                = MAV_CMD_DO_MOUNT_CONTROL;
    comm.confirmation     	= true;
    comm.param1 = pitch_deg;
    comm.param2 = roll_deg;
    comm.param3 = yaw_deg;
    comm.param7 = (float) MAV_MOUNT_MODE_MAVLINK_TARGETING;
    mavlink_message_t message;
    mavlink_msg_command_long_encode(_sysid, MAV_COMP_ID_SYSTEM_CONTROL, &message, &comm);
    buf_len = mavlink_msg_to_send_buffer(buf, &message);
    printf("  send buffer len = %d\n", buf_len);

    /*if ( buf_len <= write_buf->space() ) {
        write_buf->write((uint8_t *)buf, buf_len);
        }*/
    
    uint16_t result = _port->write((uint8_t *)&buf, buf_len);
    printf("  bytes written = %d\n", result);

    // store time of send
    _last_send = AP_HAL::millis();
}
