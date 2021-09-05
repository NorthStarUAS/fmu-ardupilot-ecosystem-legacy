/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include "setup_board.h"

#include "nav_mgr.h"            // reset ekf
#include "nav/nav_constants.h"
#include "sensors/imu_mgr.h"            // reset gyros
#include "sensors/pilot.h"              // update_ap()
#include "serial_link.h"
#include "rc_messages.h"
#include "rcfmu_messages.h"     // fixme: work towards deprecating

#include "gcs_link.h"

#include <AP_HAL/AP_HAL.h>

gcs_link_t::gcs_link_t() {}
gcs_link_t::~gcs_link_t() {}

void gcs_link_t::init() {
    config_node = PropertyNode("/config");
    effector_node = PropertyNode("/effectors");
    nav_node = PropertyNode("/filters/nav");
    airdata_node = PropertyNode("/sensors/airdata");
    gps_node = PropertyNode("/sensors/gps");
    imu_node = PropertyNode("/sensors/imu");
    power_node = PropertyNode("/sensors/power");
    pilot_node = PropertyNode("/pilot");
    
    // serial.open(DEFAULT_BAUD, hal.serial(0)); // usb/console
    // serial.open(DEFAULT_BAUD, hal.serial(1)); // telemetry 1
    serial.open(TELEMETRY_BAUD, hal.serial(2)); // telemetry 2

    airdata_limiter = RateLimiter(2);
    gps_limiter = RateLimiter(2.5);
    imu_limiter = RateLimiter(4);
    nav_limiter = RateLimiter(10);
    nav_metrics_limiter = RateLimiter(0.5);
    pilot_limiter = RateLimiter(4);
}

void gcs_link_t::update() {
    if ( pilot_limiter.update() ) {
        output_counter += write_pilot();
    }
    if ( gps_limiter.update() ) {
        output_counter += write_gps();
    }
    if ( airdata_limiter.update() ) {
        output_counter += write_airdata();
    }
    //output_counter += write_power();
    // do a little extra dance with the return value because
    // write_status_info() can reset output_counter (but
    // that gets ignored if we do the math in one step)
    //uint8_t result = write_status_info();
    //output_counter += result;
    if ( nav_limiter.update() ) {
        output_counter += write_nav();
    } else if ( nav_metrics_limiter.update() ) {
        output_counter += write_nav_metrics();
    }
    if ( imu_limiter.update() ) {
        output_counter += write_imu();
    }
}

bool gcs_link_t::parse_message( uint8_t id, uint8_t *buf, uint8_t message_size )
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
int gcs_link_t::write_ack( uint8_t command_id, uint8_t subcommand_id )
{
    static rcfmu_message::command_ack_t ack;
    ack.command_id = command_id;
    ack.subcommand_id = subcommand_id;
    ack.pack();
    return serial.write_packet( ack.id, ack.payload, ack.len);
}


// output a binary representation of the pilot manual (rc receiver) data
int gcs_link_t::write_pilot()
{
    static rc_message::pilot_v4_t pilot_msg;
    pilot_msg.props2msg(pilot_node);
    pilot_msg.pack();
    return serial.write_packet( pilot_msg.id, pilot_msg.payload, pilot_msg.len);
}

// output a binary representation of the IMU data (note: scaled to 16bit values)
int gcs_link_t::write_imu()
{
    static rc_message::imu_v6_t imu_msg;
    imu_msg.props2msg(imu_node);
    imu_msg.pack();
    return serial.write_packet( imu_msg.id, imu_msg.payload, imu_msg.len );
}

// output a binary representation of the GPS data
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

// output a binary representation of the Nav data
int gcs_link_t::write_nav()
{
    static rc_message::nav_v6_t nav_msg;
    nav_msg.props2msg(nav_node);
    nav_msg.pack();
    return serial.write_packet( nav_msg.id, nav_msg.payload, nav_msg.len );
}

// output a binary representation of the Nav data
int gcs_link_t::write_nav_metrics()
{
    static rc_message::nav_metrics_v6_t metrics_msg;
    metrics_msg.props2msg(nav_node);
    metrics_msg.metrics_millis = nav_node.getUInt("millis");
    metrics_msg.pack();
    return serial.write_packet( metrics_msg.id, metrics_msg.payload, metrics_msg.len );
}

// output a binary representation of the barometer data
int gcs_link_t::write_airdata()
{
    static rc_message::airdata_v8_t air_msg;
    air_msg.props2msg(airdata_node);
    air_msg.pack();
    return serial.write_packet( air_msg.id, air_msg.payload, air_msg.len );
}

// output a binary representation of various volt/amp sensors
int gcs_link_t::write_power()
{
    static rcfmu_message::power_t power1;
    power1.avionics_v = power_node.getDouble("avionics_v");
    power1.int_main_v = power_node.getDouble("battery_volts");
    power1.ext_main_amp = power_node.getDouble("battery_amps");
    power1.pack();
    return serial.write_packet( power1.id, power1.payload, power1.len );
}

// output a binary representation of various status and config information
int gcs_link_t::write_status_info()
{
    static uint32_t write_millis = AP_HAL::millis();
    static rcfmu_message::status_t status;

    // This info is static or slow changing so we don't need to send
    // it at a high rate.
    static int counter = 0;
    if ( counter > 0 ) {
        counter--;
        return 0;
    } else {
        counter = MASTER_HZ * 1 - 1; // a message every 1 seconds (-1 so we aren't off by one frame) 
    }

    status.serial_number = config_node.getInt("serial_number");
    status.firmware_rev = FIRMWARE_REV;
    status.master_hz = MASTER_HZ;
    status.baud = TELEMETRY_BAUD;

    // estimate sensor output byte rate
    unsigned long current_time = AP_HAL::millis();
    unsigned long elapsed_millis = current_time - write_millis;
    unsigned long byte_rate = output_counter * 1000 / elapsed_millis;
    write_millis = current_time;
    output_counter = 0;
    status.byte_rate = byte_rate;
    status.timer_misses = main_loop_timer_misses;

    status.pack();
    return serial.write_packet( status.id, status.payload, status.len );
}

void gcs_link_t::read_commands() {
    while ( serial.update() ) {
        parse_message( serial.pkt_id, serial.payload, serial.pkt_len );
    }
}
