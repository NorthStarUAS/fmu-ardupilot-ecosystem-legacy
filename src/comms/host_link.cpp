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

#include "host_link.h"

void host_link_t::init() {
    config_node = PropertyNode("/config");
    config_nav_node = PropertyNode("/config/nav"); // after config.init()
    effector_node = PropertyNode("/effectors");
    nav_node = PropertyNode("/filters/nav");
    active_node = PropertyNode("/task/route/active");
    airdata_node = PropertyNode("/sensors/airdata");
    ap_node = PropertyNode("/autopilot");
    circle_node = PropertyNode("/task/circle/active");
    gps_node = PropertyNode("/sensors/gps");
    home_node = PropertyNode("/task/home");
    imu_node = PropertyNode("/sensors/imu");
    power_node = PropertyNode("/sensors/power");
    pilot_node = PropertyNode("/pilot");
    route_node = PropertyNode("/task/route");
    status_node = PropertyNode("/status");
    targets_node = PropertyNode("/autopilot/targets");
    task_node = PropertyNode("/task");
    
    // serial.open(HOST_BAUD, hal.serial(0)); // usb/console
    serial.open(HOST_BAUD, hal.serial(1)); // telemetry 1
    // serial.open(HOST_BAUD, hal.serial(2)); // telemetry 2
    
    nav_metrics_limiter = RateLimiter(0.5);
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
        if ( nav_metrics_limiter.update() ) {
            output_counter += write_nav_metrics();
        }
    }
    // write imu message last: used as an implicit end of data
    // frame marker.
    output_counter += write_imu();
}

bool host_link_t::parse_message( uint8_t id, uint8_t *buf, uint8_t message_size )
{
    bool result = false;

    // console->print("message id = "); console->print(id); console->print(" len = "); console->println(message_size);
    
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
        console->printf("received command: %s\n", msg.message.c_str());
        uint8_t command_result = 0;
        if ( msg.message == "zero_gyros" ) {
            imu_mgr.gyros_calibrated = 0;   // start state
            command_result = 1;
        } else if ( msg.message == "reset_ekf" ) {
            nav_mgr.reinit();
            command_result = 1;
        } else {
            console->printf("unknown message\n");
        }
        write_ack( msg.sequence_num, command_result );
        result = true;
    } else if ( id == rc_message::ap_targets_v1_id ) {
        rc_message::ap_targets_v1_t ap_msg;
        ap_msg.unpack(buf, message_size);
        ap_msg.msg2props(targets_node);
    } else if ( id == rc_message::mission_v1_id ) {
        // this is the messy message
        rc_message::mission_v1_t mission;
        mission.unpack(buf, message_size);
        if ( message_size == mission.len ) {
            status_node.setDouble("flight_timer", mission.flight_timer);
            task_node.setString("current_task", mission.task_name);
            task_node.setInt("task_attribute", mission.task_attribute);
            active_node.setInt("route_size", mission.route_size);
            route_node.setInt("target_waypoint_idx", mission.target_waypoint_idx);
            double wp_lon = mission.wp_longitude_raw / 10000000.0l;
            double wp_lat = mission.wp_latitude_raw / 10000000.0l;
            int wp_index = mission.wp_index;
            PropertyNode wp_node;
            if ( mission.route_size != active_node.getInt("route_size") ) {
                // route size change, zero all the waypoint coordinates
                for ( int i = 0; i < active_node.getInt("route_size"); i++ ) {
                    string wp_path = "wpt/" + std::to_string(i);
                    wp_node = active_node.getChild(wp_path.c_str());
                    wp_node.setDouble("longitude_deg", 0);
                    wp_node.setDouble("latitude_deg", 0);
                }
            }
            if ( wp_index < mission.route_size ) {
                string wp_path = "wpt/" + std::to_string(wp_index);
                wp_node = active_node.getChild(wp_path.c_str());
                wp_node.setDouble("longitude_deg", wp_lon);
                wp_node.setDouble("latitude_deg", wp_lat);
            } else if ( wp_index == 65534 ) {
                circle_node.setDouble("longitude_deg", wp_lon);
                circle_node.setDouble("latitude_deg", wp_lat);
                circle_node.setDouble("radius_m", mission.task_attribute / 10.0);
            } else if ( wp_index == 65535 ) {
                home_node.setDouble("longitude_deg", wp_lon);
                home_node.setDouble("latitude_deg", wp_lat);
            }
            result = true;
        }
    } else {
        console->printf("unknown message id: %d len: %d\n", id, message_size);
    }
    return result;
}

// output an acknowledgement of a message received
int host_link_t::write_ack( uint16_t sequence_num, uint8_t result )
{
    static rc_message::ack_v1_t ack;
    ack.sequence_num = sequence_num;
    ack.result = result;
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
