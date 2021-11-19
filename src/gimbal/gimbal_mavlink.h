/*
  SToRM32 mount backend class
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>

#include "props2.h"

class gimbal_mavlink_t {

public:
    // Constructor
    gimbal_mavlink_t() {}

    // init - performs any required initialisation for this instance
    void init();

    // read messages from the gimbal
    void read_messages();

    // send the mavlink heartbeat message periodically
    void send_heartbeat();
    void set_imu_rate();
    
    // test: send set gimbal model message
    void set_gimbal_mode();
    void set_mount_configure();
    
    // update mount position - should be called periodically
    void update();

private:

    // send_do_mount_control - send a COMMAND_LONG containing a do_mount_control message
    void send_do_mount_control(float pitch_deg, float roll_deg, float yaw_deg);

    // internal variables
    bool _found_gimbal = false;     // true once the driver has been initialised
    bool _receiving_imu = false;    // true when we start getting imu messages
    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal
    uint32_t _last_send;            // system time of last do_mount_control sent to gimbal
    uint32_t _last_heartbeat;       // ssytem tim eof last heartbeat
    AP_HAL::UARTDriver *_port;

    PropertyNode mount_node;
    PropertyNode pilot_node;
};
