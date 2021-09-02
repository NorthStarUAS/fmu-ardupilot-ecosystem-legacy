#pragma once

#include "props2.h"
#include "serial_link.h"

class gcs_link_t {
public:
    // Serial = usb, Serial1 connects to /dev/ttyO4 on beaglebone in
    // aura-v2 and marmot-v1 hardware
    SerialLink serial;
    unsigned long output_counter = 0;
    int main_loop_timer_misses = 0; // performance sanity check

    void init();
    int write_ack( uint8_t command_id, uint8_t subcommand_id );
    int write_pilot_in();
    int write_imu();
    int write_gps();
    int write_nav();
    int write_airdata();
    int write_power();
    int write_status_info();
    bool parse_message( uint8_t id, uint8_t *buf, uint8_t message_size );
    void read_commands();
    
private:
    PropertyNode config_node;
    PropertyNode effector_node;
    PropertyNode nav_node;
    PropertyNode airdata_node;
    PropertyNode gps_node;
    PropertyNode imu_node;
    PropertyNode pilot_node;
    PropertyNode power_node;
    unsigned long int gps_last_millis = 0;
};
