#pragma once

#include "props2.h"
#include "serial_link.h"
#include "util/ratelimiter.h"

class gcs_link_t {

public:

    gcs_link_t();
    ~gcs_link_t();
    
    // Serial = usb, Serial1 connects to /dev/ttyO4 on beaglebone in
    // aura-v2 and marmot-v1 hardware
    SerialLink serial;
    unsigned long output_counter = 0;
    int main_loop_timer_misses = 0; // performance sanity check

    void init();
    void update();
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
    
    int write_ack( uint8_t command_id, uint8_t subcommand_id );
    int write_pilot();
    int write_imu();
    int write_gps();
    int write_nav();
    int write_nav_metrics();
    int write_airdata();
    int write_power();
    int write_status_info();
    bool parse_message( uint8_t id, uint8_t *buf, uint8_t message_size );

    RateLimiter airdata_limiter;
    RateLimiter ap_limiter;
    RateLimiter eff_limiter;
    RateLimiter gps_limiter;
    RateLimiter health_limiter;
    RateLimiter imu_limiter;
    RateLimiter nav_limiter;
    RateLimiter nav_metrics_limiter;
    RateLimiter pilot_limiter;
};
