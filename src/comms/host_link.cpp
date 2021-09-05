/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include "setup_board.h"

// #include "airdata.h"
#include "nav_mgr.h"            // reset ekf
#include "sensors/imu_mgr.h"            // reset gyros
#include "sensors/pilot.h"              // update_ap()
#include "nav/nav_constants.h"
#include "serial_link.h"
#include "rc_messages.h"
#include "rcfmu_messages.h"     // fixme: work towards deprecating

#include "host_link.h"

void host_link_t::init() {
    config_node = PropertyNode("/config");
    config_nav_node = PropertyNode("/config/nav"); // after config.init()
    effector_node = PropertyNode("/effectors");
    nav_node = PropertyNode("/filters/nav");
    airdata_node = PropertyNode("/sensors/airdata");
    gps_node = PropertyNode("/sensors/gps");
    imu_node = PropertyNode("/sensors/imu");
    power_node = PropertyNode("/sensors/power");
    pilot_node = PropertyNode("/pilot");
    status_node = PropertyNode("/status");
    
    // serial.open(HOST_BAUD, hal.serial(0)); // usb/console
    serial.open(HOST_BAUD, hal.serial(1)); // telemetry 1
    // serial.open(HOST_BAUD, hal.serial(2)); // telemetry 2
    
    status_limiter = RateLimiter(0.5);
}

void host_link_t::update() {
    output_counter += write_pilot();
    output_counter += write_gps();
    output_counter += write_airdata();
    output_counter += write_power();
    if ( status_limiter.update() ) {
        int len = write_status();
        output_counter = len;   // start over counting bytes
    }
    if ( config_nav_node.getString("select") != "none" ) {
        output_counter += write_nav();
        output_counter += write_nav_metrics();
    }
    // write imu message last: used as an implicit end of data
    // frame marker.
    output_counter += write_imu();
}

bool host_link_t::parse_message( uint8_t id, uint8_t *buf, uint8_t message_size )
{
    bool result = false;

    // console->print("message id = "); console->print(id); console->print(" len = "); console->println(message_size);
    
    if ( id == rcfmu_message::command_inceptors_id ) {
        static rcfmu_message::command_inceptors_t inceptors;
        inceptors.unpack(buf, message_size);
        if ( message_size == inceptors.len ) {
            pilot.update_ap(&inceptors);
            result = true;
        }
    // example of receiving a config message, doing something, and
    // replying with an ack
    // } else if ( id == rcfmu_message::config_airdata_id ) {
    //     config.airdata_cfg.unpack(buf, message_size);
    //     if ( message_size == config.airdata_cfg.len ) {
    //         console->printf("received new airdata config\n");
    //         console->printf("Swift barometer on I2C: 0x%X\n",
    //                         config.airdata_cfg.swift_baro_addr);
    //         config.write_storage();
    //         write_ack( id, 0 );
    //         result = true;
    //     }
    } else if ( id == rcfmu_message::command_zero_gyros_id && message_size == 1 ) {
        console->printf("received zero gyros command\n");
        imu_mgr.gyros_calibrated = 0;   // start state
        write_ack( id, 0 );
        result = true;
    } else if ( id == rcfmu_message::command_reset_ekf_id && message_size == 1 ) {
        console->printf("received reset ekf command\n");
        nav_mgr.reinit();
        write_ack( id, 0 );
        result = true;
    } else {
        console->printf("unknown message id: %d len: %d\n", id, message_size);
    }
    return result;
}


// output an acknowledgement of a message received
int host_link_t::write_ack( uint8_t command_id, uint8_t subcommand_id )
{
    static rcfmu_message::command_ack_t ack;
    ack.command_id = command_id;
    ack.subcommand_id = subcommand_id;
    ack.pack();
    return serial.write_packet( ack.id, ack.payload, ack.len);
}


// output a binary representation of the pilot manual (rc receiver) data
int host_link_t::write_pilot()
{
    static rc_message::pilot_v4_t pilot_msg;
    pilot_msg.props2msg(pilot_node);
    pilot_msg.pack();
    return serial.write_packet( pilot_msg.id, pilot_msg.payload, pilot_msg.len);
}

// output a binary representation of the IMU data (note: scaled to 16bit values)
int host_link_t::write_imu()
{
    static rc_message::imu_v6_t imu_msg;
    imu_msg.props2msg(imu_node);
    imu_msg.pack();
    int result = serial.write_packet( imu_msg.id, imu_msg.payload, imu_msg.len );
    return result;
}

// output a binary representation of the GPS data
int host_link_t::write_gps()
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

// output a binary representation of the Nav data (and metrics)
int host_link_t::write_nav()
{
    static rc_message::nav_v6_t nav_msg;
    nav_msg.props2msg(nav_node);
    nav_msg.pack();
    return serial.write_packet( nav_msg.id, nav_msg.payload, nav_msg.len );
}

int host_link_t::write_nav_metrics()
{
    static rc_message::nav_metrics_v6_t metrics_msg;
    metrics_msg.props2msg(nav_node);
    metrics_msg.pack();
    return serial.write_packet( metrics_msg.id, metrics_msg.payload, metrics_msg.len );
}

// output a binary representation of the barometer data
int host_link_t::write_airdata()
{
    static rc_message::airdata_v8_t air_msg;
    air_msg.props2msg(airdata_node);
    air_msg.pack();
    return serial.write_packet( air_msg.id, air_msg.payload, air_msg.len );
}

// output a binary representation of various volt/amp sensors
int host_link_t::write_power()
{
    static rc_message::power_v1_t power_msg;
    power_msg.props2msg(power_node);
    power_msg.pack();
    return serial.write_packet( power_msg.id, power_msg.payload, power_msg.len );
}

// output a binary representation of various status and config information
int host_link_t::write_status()
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

void host_link_t::read_commands() {
    while ( serial.update() ) {
        parse_message( serial.pkt_id, serial.payload, serial.pkt_len );
    }
}
