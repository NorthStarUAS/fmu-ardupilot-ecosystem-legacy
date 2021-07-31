/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include "setup_board.h"

#include "airdata.h"
#include "imu_mgr.h"            // reset gyros
#include "nav_mgr.h"            // reset ekf
#include "pilot.h"              // update_ap()
#include "nav/nav_constants.h"
#include "serial_link.h"
#include "rcfmu_messages.h"

#include "comms.h"

#include <AP_HAL/AP_HAL.h>

void comms_t::init() {
    config_node = PropertyNode("/config");
    effector_node = PropertyNode("/effectors");
    nav_node = PropertyNode("/filters/nav");
    airdata_node = PropertyNode("/sensors/airdata");
    gps_node = PropertyNode("/sensors/gps");
    imu_node = PropertyNode("/sensors/imu");
    power_node = PropertyNode("/sensors/power");
    pilot_node = PropertyNode("/pilot");
    
    // serial.open(DEFAULT_BAUD, hal.serial(0)); // usb/console
    serial.open(DEFAULT_BAUD, hal.serial(1)); // telemetry 1
    // serial.open(DEFAULT_BAUD, hal.serial(2)); // telemetry 2
}

bool comms_t::parse_message_bin( uint8_t id, uint8_t *buf, uint8_t message_size )
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
    //         write_ack_bin( id, 0 );
    //         result = true;
    //     }
    } else if ( id == rcfmu_message::command_zero_gyros_id && message_size == 1 ) {
        console->printf("received zero gyros command\n");
        imu_mgr.gyros_calibrated = 0;   // start state
        write_ack_bin( id, 0 );
        result = true;
    } else if ( id == rcfmu_message::command_reset_ekf_id && message_size == 1 ) {
        console->printf("received reset ekf command\n");
        nav_mgr.reinit();
        write_ack_bin( id, 0 );
        result = true;
    } else {
        console->printf("unknown message id: %d len: %d\n", id, message_size);
    }
    return result;
}


// output an acknowledgement of a message received
int comms_t::write_ack_bin( uint8_t command_id, uint8_t subcommand_id )
{
    static rcfmu_message::command_ack_t ack;
    ack.command_id = command_id;
    ack.subcommand_id = subcommand_id;
    ack.pack();
    return serial.write_packet( ack.id, ack.payload, ack.len);
}


// output a binary representation of the pilot manual (rc receiver) data
int comms_t::write_pilot_in_bin()
{
    static rcfmu_message::pilot_t pilot1;

    if (rcfmu_message::sbus_channels > MAX_RCIN_CHANNELS) {
        return 0;
    }
    
    // receiver data
    for ( int i = 0; i < rcfmu_message::sbus_channels; i++ ) {
        pilot1.channel[i] = pilot_node.getDouble("manual", i);
    }

    // flags
    pilot1.flags = pilot_node.getBool("failsafe");
    
    pilot1.pack();
    return serial.write_packet( pilot1.id, pilot1.payload, pilot1.len);
}

void comms_t::write_pilot_in_ascii()
{
    // pilot (receiver) input data
    if ( pilot_node.getBool("failsafe") ) {
        console->printf("FAILSAFE! ");
    }
    if ( pilot_node.getBool("ap_enabled") ) {
        console->printf("(Auto) ");
    } else {
        console->printf("(Manual) ");
    }
    if ( pilot_node.getBool("throttle_safety") ) {
        console->printf("(Throttle safety) ");
    } else {
        console->printf("(Throttle enable) ");
    }
    for ( int i = 0; i < 8; i++ ) {
        console->printf("%.3f ", pilot_node.getDouble("manual", i));
    }
    console->printf("\n");
}

void comms_t::write_actuator_out_ascii()
{
    // actuator output
    console->printf("RCOUT:");
    for ( int i = 0; i < MAX_RCOUT_CHANNELS; i++ ) {
        console->printf("%.2f ", effector_node.getDouble("channel", i));
    }
    console->printf("\n");
}

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

// output a binary representation of the IMU data (note: scaled to 16bit values)
int comms_t::write_imu_bin()
{
    static rcfmu_message::imu_t imu1;
    imu1.props2msg(imu_node);
    imu1.pack();
    int result = serial.write_packet( imu1.id, imu1.payload, imu1.len );
    return result;
}

void comms_t::write_imu_ascii()
{
    // output imu data
    console->printf("IMU: ");
    console->printf("%.3f ", imu_node.getDouble("timestamp"));
    console->printf("%.2f ", imu_node.getDouble("p_rps"));
    console->printf("%.2f ", imu_node.getDouble("q_rps"));
    console->printf("%.2f ", imu_node.getDouble("r_rps"));
    console->printf("%.2f ", imu_node.getDouble("ax_mps2"));
    console->printf("%.2f ", imu_node.getDouble("ay_mps2"));
    console->printf("%.2f ", imu_node.getDouble("az_mps2"));
    console->printf("%.2f ", imu_node.getDouble("temp_C"));
    console->printf("\n");
}

// output a binary representation of the GPS data
int comms_t::write_gps_bin()
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

void comms_t::write_gps_ascii() {
    console->printf("GPS:");
    console->printf(" Lat: %.7f", gps_node.getDouble("latitude_deg"));
    console->printf(" Lon: %.7f", gps_node.getDouble("longitude_deg"));
    console->printf(" Alt: %.1f", gps_node.getDouble("altitude_m"));
    console->printf(" Vel: %.1f %.1f %.1f",
                    gps_node.getDouble("vn_mps"),
                    gps_node.getDouble("ve_mps"),
                    gps_node.getDouble("vd_mps"));
    console->printf(" Sat: %d", gps_node.getInt("satellites"));
    console->printf(" Fix: %d", gps_node.getInt("status"));
    console->printf(" Time: %02d:%02d:%02d ",
                    gps_node.getInt("hour"),
                    gps_node.getInt("min"),
                    gps_node.getInt("sec"));
    console->printf(" Date: %02d/%02d/%04d",
                    gps_node.getInt("month"),
                    gps_node.getInt("day"),
                    gps_node.getInt("year"));
    console->printf("\n");
}

// output a binary representation of the Nav data
int comms_t::write_nav_bin()
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

void comms_t::write_nav_ascii() {
    // values
    console->printf("Pos: %.7f, %.7f, %.2f",
                    nav_node.getDouble("latitude_rad")*R2D,
                    nav_node.getDouble("longitude_rad")*R2D,
                    nav_node.getDouble("altitude_m"));
    console->printf(" Vel: %.2f, %.2f, %.2f",
                    nav_node.getDouble("vn_mps"),
                    nav_node.getDouble("ve_mps"),
                    nav_node.getDouble("vd_mps"));
    console->printf(" Att: %.2f, %.2f, %.2f\n",
                    nav_node.getDouble("phi_rad")*R2D,
                    nav_node.getDouble("the_rad")*R2D,
                    nav_node.getDouble("psi_rad")*R2D);
}

void comms_t::write_nav_stats_ascii() {
    // covariances
    console->printf("gxb: %.2f %.2f %.2f",
                    nav_node.getDouble("p_bias"),
                    nav_node.getDouble("q_bias"),
                    nav_node.getDouble("r_bias"));
    console->printf(" axb: %.2f %.2f %.2f",
                    nav_node.getDouble("ax_bias"),
                    nav_node.getDouble("ay_bias"),
                    nav_node.getDouble("az_bias"));
    float num = 3.0;            // how many standard deviations
    console->printf(" cov pos: %.2f %.2f %.2f",
                    num * nav_node.getDouble("Pp0"),
                    num * nav_node.getDouble("Pp1"),
                    num * nav_node.getDouble("Pp2"));
    console->printf(" vel: %.2f %.2f %.2f",
                    num * nav_node.getDouble("Pv0"),
                    num * nav_node.getDouble("Pv1"),
                    num * nav_node.getDouble("Pv2"));
    console->printf(" att: %.2f %.2f %.2f\n",
                    num * nav_node.getDouble("Pa0")*R2D,
                    num * nav_node.getDouble("Pa1")*R2D,
                    num * nav_node.getDouble("Pa2")*R2D);
    if ( false ) {
        nav_node.pretty_print();
    }
}

// output a binary representation of the barometer data
int comms_t::write_airdata_bin()
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

void comms_t::write_airdata_ascii()
{
    console->printf("Baro: %.2f pa %.1f C ",
                    airdata_node.getDouble("baro_press_pa"),
                    airdata_node.getDouble("baro_tempC"));
    console->printf("Pitot: %.4f mps %.1f C %d errors\n",
                    airdata_node.getDouble("airspeed_mps"),
                    airdata_node.getDouble("temp_C"),
                    airdata_node.getUInt("error_count"));
}

// output a binary representation of various volt/amp sensors
int comms_t::write_power_bin()
{
    static rcfmu_message::power_t power1;
    power1.avionics_v = power_node.getDouble("avionics_v");
    power1.int_main_v = power_node.getDouble("battery_volts");
    power1.ext_main_amp = power_node.getDouble("battery_amps");
    power1.pack();
    return serial.write_packet( power1.id, power1.payload, power1.len );
}

void comms_t::write_power_ascii()
{
    printf("Avionics v: %.2f  Batt v: %.2f  Batt amp: %.2f\n",
           power_node.getDouble("avionics_v"),
           power_node.getDouble("battery_volts"),
           power_node.getDouble("battery_amps"));
}

// output a binary representation of various status and config information
int comms_t::write_status_info_bin()
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
    status.baud = DEFAULT_BAUD;

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

void comms_t::write_status_info_ascii()
{
    // This info is static so we don't need to send it at a high rate ... once every 10 seconds (?)
    // with an immediate message at the start.
    printf("Uptime: %d(sec)", (unsigned int)(AP_HAL::millis() / 1000));
    printf(" SN: %d", config_node.getInt("serial_number"));
    printf(" Firmware: %d", FIRMWARE_REV);
    printf(" Main loop hz: %d", MASTER_HZ);
    printf(" Baud: %d\n", DEFAULT_BAUD);
}

void comms_t::read_commands() {
    while ( serial.update() ) {
        parse_message_bin( serial.pkt_id, serial.payload, serial.pkt_len );
    }
}

// global shared instance
comms_t comms;
