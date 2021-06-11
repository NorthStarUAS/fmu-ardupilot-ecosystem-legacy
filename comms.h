#pragma once

#include "serial_link.h"

class comms_t {
public:
    // Serial = usb, Serial1 connects to /dev/ttyO4 on beaglebone in
    // aura-v2 and marmot-v1 hardware
    SerialLink serial;
    unsigned long output_counter = 0;
    int main_loop_timer_misses = 0; // performance sanity check

    void setup();
    int write_ack_bin( uint8_t command_id, uint8_t subcommand_id );
    int write_pilot_in_bin();
    void write_pilot_in_ascii();
    void write_actuator_out_ascii();
    int write_imu_bin();
    void write_imu_ascii();
    int write_gps_bin();
    void write_gps_ascii();
    int write_nav_bin();
    void write_nav_ascii();
    int write_airdata_bin();
    void write_airdata_ascii();
    int write_power_bin();
    void write_power_ascii();
    int write_status_info_bin();
    void write_status_info_ascii();
    bool parse_message_bin( uint8_t id, uint8_t *buf, uint8_t message_size );
    void read_commands();
    
private:
    unsigned long int gps_last_millis = 0;
};

extern comms_t comms;
