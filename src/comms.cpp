/* Binary I/O section: generial info ...
 * Packets start with two bytes ... START_OF_MSG0 and START_OF_MSG1
 * Following that is the packet ID
 * Following that is the packet data size (not including start bytes or check sum, just the data)
 * Following that is the actual data packet
 * Following that is a two byte check sum.  The check sum includes the packet id and size as well as the data.
 */

#include "setup_board.h"

#include "airdata.h"
#include "imu_mgr.h"
#include "mixer.h"
#include "nav_mgr.h"
#include "pilot.h"
#include "power.h"
#include "nav/nav_constants.h"
#include "serial_link.h"
#include "aura4_messages.h"

#include "comms.h"

#include <AP_HAL/AP_HAL.h>

void comms_t::setup() {
    config_node = PropertyNode("/config");
    gps_node = PropertyNode("/sensors/gps");
    imu_node = PropertyNode("/sensors/imu");
    power_node = PropertyNode("/sensors/power");
    // serial.open(DEFAULT_BAUD, hal.serial(0)); // usb/console
    serial.open(DEFAULT_BAUD, hal.serial(1)); // telemetry 1
    // serial.open(DEFAULT_BAUD, hal.serial(2)); // telemetry 2
}

bool comms_t::parse_message_bin( uint8_t id, uint8_t *buf, uint8_t message_size )
{
    bool result = false;

    // console->print("message id = "); console->print(id); console->print(" len = "); console->println(message_size);
    
    if ( id == message::command_inceptors_id ) {
        static message::command_inceptors_t inceptors;
        inceptors.unpack(buf, message_size);
        if ( message_size == inceptors.len ) {
            pilot.update_ap(&inceptors);
            result = true;
        }
    // example of receiving a config message, doing something, and
    // replying with an ack
    // } else if ( id == message::config_airdata_id ) {
    //     config.airdata_cfg.unpack(buf, message_size);
    //     if ( message_size == config.airdata_cfg.len ) {
    //         console->printf("received new airdata config\n");
    //         console->printf("Swift barometer on I2C: 0x%X\n",
    //                         config.airdata_cfg.swift_baro_addr);
    //         config.write_storage();
    //         write_ack_bin( id, 0 );
    //         result = true;
    //     }
    } else if ( id == message::command_zero_gyros_id && message_size == 1 ) {
        console->printf("received zero gyros command\n");
        imu_mgr.gyros_calibrated = 0;   // start state
        write_ack_bin( id, 0 );
        result = true;
    } else if ( id == message::command_reset_ekf_id && message_size == 1 ) {
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
    static message::command_ack_t ack;
    ack.command_id = command_id;
    ack.subcommand_id = subcommand_id;
    ack.pack();
    return serial.write_packet( ack.id, ack.payload, ack.len);
}


// output a binary representation of the pilot manual (rc receiver) data
int comms_t::write_pilot_in_bin()
{
    static message::pilot_t pilot1;

    if (message::sbus_channels > MAX_RCIN_CHANNELS) {
        return 0;
    }
    
    // receiver data
    for ( int i = 0; i < message::sbus_channels; i++ ) {
        pilot1.channel[i] = pilot.manual_inputs[i];
    }

    // flags
    pilot1.flags = pilot.failsafe;
    
    pilot1.pack();
    return serial.write_packet( pilot1.id, pilot1.payload, pilot1.len);
}

void comms_t::write_pilot_in_ascii()
{
    // pilot (receiver) input data
    if ( pilot.failsafe ) {
        console->printf("FAILSAFE! ");
    }
    if ( pilot.ap_enabled() ) {
        console->printf("(Auto) ");
    } else {
        console->printf("(Manual) ");
    }
    if ( pilot.throttle_safety() ) {
        console->printf("(Throttle safety) ");
    } else {
        console->printf("(Throttle enable) ");
    }
    for ( int i = 0; i < 8; i++ ) {
        console->printf("%.3f ", pilot.manual_inputs[i]);
    }
    console->printf("\n");
}

void comms_t::write_actuator_out_ascii()
{
    // actuator output
    console->printf("RCOUT:");
    for ( int i = 0; i < MAX_RCOUT_CHANNELS; i++ ) {
        console->printf("%.2f ", pilot.mixer.outputs[i]);
    }
    console->printf("\n");
}

static inline int32_t intround(float f) {
    return (int32_t)(f >= 0.0 ? (f + 0.5) : (f - 0.5));
}

// output a binary representation of the IMU data (note: scaled to 16bit values)
int comms_t::write_imu_bin()
{
    const float _pi = 3.14159265358979323846;
    const float _g = 9.807;
    const float _d2r = _pi / 180.0;
    
    const float _gyro_lsb_per_dps = 32767.5 / 500;  // -500 to +500 spread across 65535
    const float gyroScale = _d2r / _gyro_lsb_per_dps;
    
    const float _accel_lsb_per_dps = 32767.5 / 8;   // -4g to +4g spread across 65535
    const float accelScale = _g / _accel_lsb_per_dps;

    const float magScale = 0.01;
    const float tempScale = 0.01;
    
    static message::imu_t imu1;
    imu1.millis = imu_node.getUInt("millis");
    imu1.raw[0] = intround(imu_node.getFloat("ax_raw") / accelScale);
    imu1.raw[1] = intround(imu_node.getFloat("ay_raw") / accelScale);
    imu1.raw[2] = intround(imu_node.getFloat("az_raw") / accelScale);
    imu1.raw[3] = intround(imu_node.getFloat("hx_raw") / magScale);
    imu1.raw[4] = intround(imu_node.getFloat("hy_raw") / magScale);
    imu1.raw[5] = intround(imu_node.getFloat("hz_raw") / magScale);
    imu1.cal[0] = intround(imu_node.getFloat("ax_mps2") / accelScale);
    imu1.cal[1] = intround(imu_node.getFloat("ay_mps2") / accelScale);
    imu1.cal[2] = intround(imu_node.getFloat("az_mps2") / accelScale);
    imu1.cal[3] = intround(imu_node.getFloat("p_rps") / gyroScale);
    imu1.cal[4] = intround(imu_node.getFloat("q_rps") / gyroScale);
    imu1.cal[5] = intround(imu_node.getFloat("r_rps") / gyroScale);
    imu1.cal[6] = intround(imu_node.getFloat("hx") / magScale);
    imu1.cal[7] = intround(imu_node.getFloat("hy") / magScale);
    imu1.cal[8] = intround(imu_node.getFloat("hz") / magScale);
    imu1.cal[9] = intround(imu_node.getFloat("tempC") / tempScale);
    imu1.pack();
    int result = serial.write_packet( imu1.id, imu1.payload, imu1.len );
    return result;
}

void comms_t::write_imu_ascii()
{
    // output imu data
    console->printf("IMU: ");
    console->printf("%.3f ", imu_node.getFloat("timestamp"));
    console->printf("%.2f ", imu_node.getFloat("p_rps"));
    console->printf("%.2f ", imu_node.getFloat("q_rps"));
    console->printf("%.2f ", imu_node.getFloat("r_rps"));
    console->printf("%.2f ", imu_node.getFloat("ax_mps2"));
    console->printf("%.2f ", imu_node.getFloat("ay_mps2"));
    console->printf("%.2f ", imu_node.getFloat("az_mps2"));
    console->printf("%.2f ", imu_node.getFloat("tempC"));
    console->printf("\n");
}

// output a binary representation of the GPS data
int comms_t::write_gps_bin()
{
    if ( gps_node.getUInt("millis") != gps_last_millis ) {
#if 0 // fixme
        gps_last_millis = gps_node.getUInt("millis");
        return serial.write_packet( message::aura_nav_pvt_id,
                                    (uint8_t *)(&(gps_mgr.gps_data)),
                                    sizeof(gps_mgr.gps_data) );
#else
        return 0;
#endif
    } else {
        return 0;
    }
}

void comms_t::write_gps_ascii() {
    console->printf("GPS:");
    console->printf(" Lat: %.7f", gps_node.getDouble("latitude_deg"));
    console->printf(" Lon: %.7f", gps_node.getDouble("longitude_deg"));
    console->printf(" Alt: %.1f", gps_node.getFloat("altitude_m"));
    console->printf(" Vel: %.1f %.1f %.1f",
                    gps_node.getFloat("vn_mps"),
                    gps_node.getFloat("ve_mps"),
                    gps_node.getFloat("vd_mps"));
    console->printf(" Sat: %d", gps_node.getInt("satellites"));
    console->printf(" Fix: %d", gps_node.getInt("status"));
    console->printf(" Time: %02d:%02d:%02d ",
                    gps_node.getInt("hour"),
                    gps_node.getInt("min"),
                    gps_node.getInt("sec"));
    console->printf(" Date: %02d/%02d/%04d",
                    gps_node.getInt("mon"),
                    gps_node.getInt("day"),
                    gps_node.getInt("year"));
    console->printf("\n");
}

// output a binary representation of the Nav data
int comms_t::write_nav_bin()
{
    static message::ekf_t nav_msg;
    nav_msg.millis = imu_node.getUInt("millis"); // fixme?
    nav_msg.lat_rad = nav_mgr.data.lat;
    nav_msg.lon_rad = nav_mgr.data.lon;
    nav_msg.altitude_m = nav_mgr.data.alt;
    nav_msg.vn_ms = nav_mgr.data.vn;
    nav_msg.ve_ms = nav_mgr.data.ve;
    nav_msg.vd_ms = nav_mgr.data.vd;
    nav_msg.phi_rad = nav_mgr.data.phi;
    nav_msg.the_rad = nav_mgr.data.the;
    nav_msg.psi_rad = nav_mgr.data.psi;
    nav_msg.p_bias = nav_mgr.data.gbx;
    nav_msg.q_bias = nav_mgr.data.gby;
    nav_msg.r_bias = nav_mgr.data.gbz;
    nav_msg.ax_bias = nav_mgr.data.abx;
    nav_msg.ay_bias = nav_mgr.data.aby;
    nav_msg.az_bias = nav_mgr.data.abz;
    float max_pos_cov = nav_mgr.data.Pp0;
    if ( nav_mgr.data.Pp1 > max_pos_cov ) { max_pos_cov = nav_mgr.data.Pp1; }
    if ( nav_mgr.data.Pp2 > max_pos_cov ) { max_pos_cov = nav_mgr.data.Pp2; }
    if ( max_pos_cov > 655.0 ) { max_pos_cov = 655.0; }
    nav_msg.max_pos_cov = max_pos_cov;
    float max_vel_cov = nav_mgr.data.Pv0;
    if ( nav_mgr.data.Pv1 > max_vel_cov ) { max_vel_cov = nav_mgr.data.Pv1; }
    if ( nav_mgr.data.Pv2 > max_vel_cov ) { max_vel_cov = nav_mgr.data.Pv2; }
    if ( max_vel_cov > 65.5 ) { max_vel_cov = 65.5; }
    nav_msg.max_vel_cov = max_vel_cov;
    float max_att_cov = nav_mgr.data.Pa0;
    if ( nav_mgr.data.Pa1 > max_att_cov ) { max_att_cov = nav_mgr.data.Pa1; }
    if ( nav_mgr.data.Pa2 > max_att_cov ) { max_att_cov = nav_mgr.data.Pa2; }
    if ( max_att_cov > 6.55 ) { max_vel_cov = 6.55; }
    nav_msg.max_att_cov = max_att_cov;
    nav_msg.status = nav_mgr.status;
    nav_msg.pack();
    return serial.write_packet( nav_msg.id, nav_msg.payload, nav_msg.len );
}

void comms_t::write_nav_ascii() {
    if ( true ) {
        // values
        console->printf("Pos: %.7f, %.7f, %.2f",
                        nav_mgr.data.lat*R2D, nav_mgr.data.lon*R2D,nav_mgr.data.alt);
        console->printf(" Vel: %.2f, %.2f, %.2f",
                        nav_mgr.data.vn, nav_mgr.data.ve, nav_mgr.data.vd);
        console->printf(" Att: %.2f, %.2f, %.2f\n",
                        nav_mgr.data.phi*R2D, nav_mgr.data.the*R2D, nav_mgr.data.psi*R2D);
    } else {
        // covariances
        float num = 3.0;        // how many standard deviations
        console->printf("cov pos: %.2f, %.2f, %.2f",
                        num * nav_mgr.data.Pp0,
                        num * nav_mgr.data.Pp1,
                        num * nav_mgr.data.Pp2);
        console->printf(" vel: %.2f, %.2f, %.2f",
                        num * nav_mgr.data.Pv0,
                        num * nav_mgr.data.Pv1,
                        num * nav_mgr.data.Pv2);
        console->printf(" att: %.2f, %.2f, %.2f\n",
                        num * nav_mgr.data.Pa0*R2D,
                        num * nav_mgr.data.Pa1*R2D,
                        num * nav_mgr.data.Pa2*R2D);
    }
}

// output a binary representation of the barometer data
int comms_t::write_airdata_bin()
{
    static message::airdata_t airdata1;
    airdata1.baro_press_pa = airdata.baro_press;
    airdata1.baro_temp_C = airdata.baro_temp;
    airdata1.baro_hum = airdata.baro_hum;
    airdata1.ext_diff_press_pa = airdata.diffPress_pa;
    airdata1.ext_static_press_pa = 0.0; // fixme!
    airdata1.ext_temp_C = airdata.temp_C;
    airdata1.error_count = airdata.error_count;
    airdata1.pack();
    return serial.write_packet( airdata1.id, airdata1.payload, airdata1.len );
}

void comms_t::write_airdata_ascii()
{
#if 0
    console->print("Barometer: ");
    console->print(airdata.baro_press, 2); console->print(" (st pa) ");
    console->print(airdata.baro_temp, 2); console->print(" (C) ");
    console->print(airdata.baro_hum, 1); console->print(" (%RH) ");
    console->print("Pitot: ");
    console->print(airdata.diffPress_pa, 4); console->print(" (diff pa) ");
    console->print(airdata.temp_C, 2); console->print(" (C) ");
    console->print(airdata.error_count); console->print(" (errors) ");
    console->println();
#endif
}

// output a binary representation of various volt/amp sensors
int comms_t::write_power_bin()
{
    static message::power_t power1;
    power1.avionics_v = power_node.getFloat("avionics_v");
    power1.int_main_v = power_node.getFloat("battery_volts");
    power1.ext_main_amp = power_node.getFloat("battery_amps");
    power1.pack();
    return serial.write_packet( power1.id, power1.payload, power1.len );
}

void comms_t::write_power_ascii()
{
    console->printf("Avionics v: %.2f  Batt v: %.2f  Batt amp: %.2f\n",
                    power_node.getFloat("avionics_v"),
                    power_node.getFloat("battery_volts"),
                    power_node.getFloat("battery_amps"));
}

// output a binary representation of various status and config information
int comms_t::write_status_info_bin()
{
    static uint32_t write_millis = AP_HAL::millis();
    static message::status_t status;

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
    console->printf("Uptime: %ld(sec)", AP_HAL::millis() / 1000);
    console->printf(" SN: %d", config_node.getInt("serial_number"));
    console->printf(" Firmware: %d", FIRMWARE_REV);
    console->printf(" Main loop hz: %d", MASTER_HZ);
    console->printf(" Baud: %d\n", DEFAULT_BAUD);
}

void comms_t::read_commands() {
    while ( serial.update() ) {
        parse_message_bin( serial.pkt_id, serial.payload, serial.pkt_len );
    }
}

// global shared instance
comms_t comms;
