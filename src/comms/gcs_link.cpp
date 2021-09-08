/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include "setup_board.h"

#include "nav_mgr.h"                    // reset ekf
#include "nav/nav_constants.h"
#include "sensors/imu_mgr.h"            // reset gyros
#include "sensors/pilot.h"              // update_ap()
#include "serial_link.h"
#include "rc_messages.h"

#include "gcs_link.h"

#include <AP_HAL/AP_HAL.h>

gcs_link_t::gcs_link_t() {}
gcs_link_t::~gcs_link_t() {}

void gcs_link_t::init() {
    config_node = PropertyNode("/config");
    effector_node = PropertyNode("/effectors");
    nav_node = PropertyNode("/filters/nav");
    airdata_node = PropertyNode("/sensors/airdata");
    ap_node = PropertyNode("/autopilot");
    gps_node = PropertyNode("/sensors/gps");
    imu_node = PropertyNode("/sensors/imu");
    power_node = PropertyNode("/sensors/power");
    pilot_node = PropertyNode("/pilot");
    status_node = PropertyNode("/status");
    targets_node = PropertyNode("/autopilot/targets");

    // serial.open(DEFAULT_BAUD, hal.serial(0)); // usb/console
    // serial.open(DEFAULT_BAUD, hal.serial(1)); // telemetry 1
    serial.open(TELEMETRY_BAUD, hal.serial(2)); // telemetry 2

    airdata_limiter = RateLimiter(2);
    ap_limiter = RateLimiter(2);
    gps_limiter = RateLimiter(2.5);
    imu_limiter = RateLimiter(4);
    nav_limiter = RateLimiter(10);
    nav_metrics_limiter = RateLimiter(0.5);
    pilot_limiter = RateLimiter(4);
    power_limiter = RateLimiter(1);
    status_limiter = RateLimiter(0.1);
}

void gcs_link_t::update() {
    if ( airdata_limiter.update() ) {
        output_counter += write_airdata();
    }
    if ( ap_limiter.update() ) {
        output_counter += write_ap();
    }
    if ( gps_limiter.update() ) {
        output_counter += write_gps();
    }
    if ( imu_limiter.update() ) {
        output_counter += write_imu();
    }
    if ( nav_limiter.update() ) {
        output_counter += write_nav();
    } else if ( nav_metrics_limiter.update() ) {
        output_counter += write_nav_metrics();
    }
    if ( pilot_limiter.update() ) {
        output_counter += write_pilot();
    }
    if ( power_limiter.update() ) {
        output_counter += write_power();
    }
    if ( status_limiter.update() ) {
        int len = write_status();
        output_counter = len;   // start over counting bytes
    }
}

bool gcs_link_t::parse_message( uint8_t id, uint8_t *buf, uint8_t message_size )
{
    bool result = false;
    //console->printf("message id: %d  len: %d\n", id, message_size);
    if ( id == rc_message::inceptors_v4_id ) {
        static rc_message::inceptors_v4_t inceptors;
        inceptors.unpack(buf, message_size);
        if ( message_size == inceptors.len ) {
            pilot.update_ap(&inceptors);
            result = true;
        }
    } else if ( id == rc_message::command_v1_id ) {
        rc_message::command_v1_t msg;
        msg.unpack(buf, message_size);
        // console->printf("received command: %s\n", msg.message.c_str());
        uint8_t command_result = 0;
        if ( msg.message == "hb" ) {
            command_result = 1;
        } else if ( msg.message == "zero_gyros" ) {
            imu_mgr.gyros_calibrated = 0;   // start state
            command_result = 1;
        } else if ( msg.message == "reset_ekf" ) {
            nav_mgr.reinit();
            command_result = 1;
        } else if ( msg.message.substr(0, 4) == "get " ) {
            string path = msg.message.substr(4);
            // printf("cmd: get  node: %s\n", path.c_str());
            PropertyNode node(path);
            rc_message::command_v1_t reply;
            reply.sequence_num = 0;
            reply.message = "set " + path + " " + node.write_as_string();
            reply.pack();
            serial.write_packet( reply.id, reply.payload, reply.len);
            command_result = 1;
        } else {
            console->printf("unknown message: %s\n", msg.message.c_str());
        }
        write_ack( msg.sequence_num, command_result );
        result = true;
    } else {
        console->printf("unknown message id: %d len: %d\n", id, message_size);
    }
    return result;
}

// return an ack of a message received
int gcs_link_t::write_ack( uint16_t sequence_num, uint8_t result )
{
    static rc_message::ack_v1_t ack;
    ack.sequence_num = sequence_num;
    ack.result = result;
    ack.pack();
    return serial.write_packet( ack.id, ack.payload, ack.len);
}

// pilot manual (rc receiver) data
int gcs_link_t::write_pilot()
{
    static rc_message::pilot_v4_t pilot_msg;
    pilot_msg.props2msg(pilot_node);
    pilot_msg.pack();
    return serial.write_packet( pilot_msg.id, pilot_msg.payload, pilot_msg.len);
}

int gcs_link_t::write_imu()
{
    static rc_message::imu_v6_t imu_msg;
    imu_msg.props2msg(imu_node);
    imu_msg.pack();
    return serial.write_packet( imu_msg.id, imu_msg.payload, imu_msg.len );
}

int gcs_link_t::write_gps()
{
    static rc_message::gps_v5_t gps_msg;
    if ( gps_node.getUInt("millis") != gps_last_millis ) {
        gps_last_millis = gps_node.getUInt("millis");
        gps_msg.props2msg(gps_node);
        gps_msg.pack();
        return serial.write_packet( gps_msg.id, gps_msg.payload, gps_msg.len );
    } else {
        return 0;
    }
}

// nav (ekf) data
int gcs_link_t::write_nav()
{
    static rc_message::nav_v6_t nav_msg;
    nav_msg.props2msg(nav_node);
    nav_msg.pack();
    return serial.write_packet( nav_msg.id, nav_msg.payload, nav_msg.len );
}

// nav (ekf) metrics
int gcs_link_t::write_nav_metrics()
{
    static rc_message::nav_metrics_v6_t metrics_msg;
    metrics_msg.props2msg(nav_node);
    metrics_msg.metrics_millis = nav_node.getUInt("millis");
    metrics_msg.pack();
    return serial.write_packet( metrics_msg.id, metrics_msg.payload, metrics_msg.len );
}

int gcs_link_t::write_airdata()
{
    static rc_message::airdata_v8_t air_msg;
    air_msg.props2msg(airdata_node);
    air_msg.pack();
    return serial.write_packet( air_msg.id, air_msg.payload, air_msg.len );
}

// autopilot targets / status
int gcs_link_t::write_ap()
{
    rc_message::ap_targets_v1_t ap_msg;
    ap_msg.props2msg(targets_node);
    ap_msg.millis = imu_node.getUInt("millis");
    ap_msg.flags = 0;
    if ( ap_node.getBool("master_switch") ) {
        ap_msg.flags += 1; // |= (1 << 0)
    }
    if ( ap_node.getBool("pilot_pass_through") ) {
        ap_msg.flags += 2; // |= (1 << 1)
    }
    ap_msg.pack();
    return serial.write_packet( ap_msg.id, ap_msg.payload, ap_msg.len );
}

int gcs_link_t::write_power()
{
    static rc_message::power_v1_t power_msg;
    power_msg.props2msg(power_node);
    power_msg.pack();
    return serial.write_packet( power_msg.id, power_msg.payload, power_msg.len );
}

// system status
int gcs_link_t::write_status()
{
    static rc_message::status_v7_t status_msg;
    
    // estimate output byte rate
    uint32_t current_time = AP_HAL::millis();
    uint32_t elapsed_millis = current_time - bytes_last_millis;
    bytes_last_millis = current_time;
    uint32_t byte_rate = output_counter * 1000 / elapsed_millis;
    status_node.setUInt("byte_rate", byte_rate);
    
    status_msg.props2msg(status_node);
    status_msg.millis = current_time;
    status_msg.pack();
    return serial.write_packet( status_msg.id, status_msg.payload, status_msg.len );
}

void gcs_link_t::read_commands() {
    while ( serial.update() ) {
        parse_message( serial.pkt_id, serial.payload, serial.pkt_len );
    }
}
