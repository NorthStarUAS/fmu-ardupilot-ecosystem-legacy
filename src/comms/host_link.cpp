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
    
    // serial.open(HOST_BAUD, hal.serial(0)); // usb/console
    serial.open(HOST_BAUD, hal.serial(1)); // telemetry 1
    // serial.open(HOST_BAUD, hal.serial(2)); // telemetry 2
}

void host_link_t::update() {
    output_counter += write_pilot_in();
    output_counter += write_gps();
    output_counter += write_airdata();
    output_counter += write_power();
    // do a little extra dance with the return value because
    // write_status_info() can reset output_counter (but
    // that gets ignored if we do the math in one step)
    uint8_t result = write_status_info();
    output_counter += result;
    if ( config_nav_node.getString("select") != "none" ) {
        output_counter += write_nav();
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
int host_link_t::write_pilot_in()
{
    static rcfmu_message::pilot_t pilot1;

    // receiver data
    for ( int i = 0; i < rcfmu_message::sbus_channels; i++ ) {
        pilot1.channel[i] = pilot_node.getDouble("channel", i);
    }

    // flags
    pilot1.flags = pilot_node.getBool("failsafe");
    
    pilot1.pack();
    return serial.write_packet( pilot1.id, pilot1.payload, pilot1.len);
}

// output a binary representation of the IMU data (note: scaled to 16bit values)
int host_link_t::write_imu()
{
    static rcfmu_message::imu_t imu1;
    imu1.props2msg(imu_node);
    imu1.pack();
    int result = serial.write_packet( imu1.id, imu1.payload, imu1.len );
    return result;
}

// output a binary representation of the GPS data
int host_link_t::write_gps()
{
    static rcfmu_message::gps_t gps_msg;
    if ( gps_node.getUInt("millis") != gps_last_millis ) {
        gps_last_millis = gps_node.getUInt("millis");
        gps_msg.millis = gps_node.getUInt("millis");
        gps_msg.unix_usec = gps_node.getUInt64("unix_usec");
        // for ( int i = 0; i < 8; i++ ) {
        //     printf("%02X ", *(uint8_t *)(&(gps_msg.unix_usec) + i));
        // }
        // printf("%ld\n", gps_msg.unix_usec);
        gps_msg.num_sats = gps_node.getInt("satellites");
        gps_msg.status = gps_node.getInt("status");
        gps_msg.latitude_raw = gps_node.getInt("latitude_raw");
        gps_msg.longitude_raw = gps_node.getInt("longitude_raw");
        gps_msg.altitude_m = gps_node.getDouble("altitude_m");
        gps_msg.vn_mps = gps_node.getDouble("vn_mps");
        gps_msg.ve_mps = gps_node.getDouble("ve_mps");
        gps_msg.vd_mps = gps_node.getDouble("vd_mps");
        gps_msg.hAcc = gps_node.getDouble("hAcc");
        gps_msg.vAcc = gps_node.getDouble("vAcc");
        gps_msg.hdop = gps_node.getDouble("hdop");
        gps_msg.vdop = gps_node.getDouble("vdop");
        gps_msg.pack();
        return serial.write_packet( gps_msg.id, gps_msg.payload, gps_msg.len );
    } else {
        return 0;
    }
}

// output a binary representation of the Nav data
int host_link_t::write_nav()
{
    static rcfmu_message::ekf_t nav_msg;
    nav_msg.millis = imu_node.getUInt("millis"); // fixme?
    nav_msg.lat_rad = nav_node.getDouble("latitude_rad");
    nav_msg.lon_rad = nav_node.getDouble("longitude_rad");
    nav_msg.altitude_m = nav_node.getDouble("altitude_m");
    nav_msg.vn_ms = nav_node.getDouble("vn_mps");
    nav_msg.ve_ms = nav_node.getDouble("ve_mps");
    nav_msg.vd_ms = nav_node.getDouble("vd_mps");
    nav_msg.phi_rad = nav_node.getDouble("phi_rad");
    nav_msg.the_rad = nav_node.getDouble("the_rad");
    nav_msg.psi_rad = nav_node.getDouble("psi_rad");
    nav_msg.p_bias = nav_node.getDouble("p_bias");
    nav_msg.q_bias = nav_node.getDouble("q_bias");
    nav_msg.r_bias = nav_node.getDouble("r_bias");
    nav_msg.ax_bias = nav_node.getDouble("ax_bias");
    nav_msg.ay_bias = nav_node.getDouble("ay_bias");
    nav_msg.az_bias = nav_node.getDouble("az_bias");
    float max_pos_cov = nav_node.getDouble("Pp0");
    if ( nav_node.getDouble("Pp1") > max_pos_cov ) { max_pos_cov = nav_node.getDouble("Pp1"); }
    if ( nav_node.getDouble("Pp2") > max_pos_cov ) { max_pos_cov = nav_node.getDouble("Pp2"); }
    if ( max_pos_cov > 655.0 ) { max_pos_cov = 655.0; }
    nav_msg.max_pos_cov = max_pos_cov;
    float max_vel_cov = nav_node.getDouble("Pv0");
    if ( nav_node.getDouble("Pv1") > max_vel_cov ) { max_vel_cov = nav_node.getDouble("Pv1"); }
    if ( nav_node.getDouble("Pv2") > max_vel_cov ) { max_vel_cov = nav_node.getDouble("Pv2"); }
    if ( max_vel_cov > 65.5 ) { max_vel_cov = 65.5; }
    nav_msg.max_vel_cov = max_vel_cov;
    float max_att_cov = nav_node.getDouble("Pa0");
    if ( nav_node.getDouble("Pa1") > max_att_cov ) { max_att_cov = nav_node.getDouble("Pa1"); }
    if ( nav_node.getDouble("Pa2") > max_att_cov ) { max_att_cov = nav_node.getDouble("Pa2"); }
    if ( max_att_cov > 6.55 ) { max_vel_cov = 6.55; }
    nav_msg.max_att_cov = max_att_cov;
    nav_msg.status = nav_node.getInt("status");
    nav_msg.pack();
    return serial.write_packet( nav_msg.id, nav_msg.payload, nav_msg.len );
}

// output a binary representation of the barometer data
int host_link_t::write_airdata()
{
    static rcfmu_message::airdata_t airdata1;
    // FIXME: proprty names
    airdata1.baro_press_pa = airdata_node.getDouble("baro_press_pa");
    airdata1.baro_temp_C = airdata_node.getDouble("baro_tempC");
    airdata1.baro_hum = 0.0;
    airdata1.ext_diff_press_pa = airdata_node.getDouble("diffPress_pa");
    airdata1.ext_static_press_pa = airdata_node.getDouble("static_press_pa"); // fixme!
    airdata1.ext_temp_C = airdata_node.getDouble("temp_C");
    airdata1.error_count = airdata_node.getDouble("error_count");
    airdata1.pack();
    return serial.write_packet( airdata1.id, airdata1.payload, airdata1.len );
}

// output a binary representation of various volt/amp sensors
int host_link_t::write_power()
{
    static rcfmu_message::power_t power1;
    power1.avionics_v = power_node.getDouble("avionics_v");
    power1.int_main_v = power_node.getDouble("battery_volts");
    power1.ext_main_amp = power_node.getDouble("battery_amps");
    power1.pack();
    return serial.write_packet( power1.id, power1.payload, power1.len );
}

// output a binary representation of various status and config information
int host_link_t::write_status_info()
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
    status.baud = HOST_BAUD;

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

void host_link_t::read_commands() {
    while ( serial.update() ) {
        parse_message( serial.pkt_id, serial.payload, serial.pkt_len );
    }
}
