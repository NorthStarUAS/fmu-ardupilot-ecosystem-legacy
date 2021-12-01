#include "comms_mgr.h"

void comms_mgr_t::init() {
    imu_node = PropertyNode("/sensors/imu");
    status_node = PropertyNode("/status");
    
    info_timer = RateLimiter(10);
    heartbeat = RateLimiter(0.1);
    tempTimer = AP_HAL::millis();
    counter = 0;
    
    //gcs_link.init(2, 57600);
    gcs_link.init(2, 115200 /*FIXME 57600*/);
    //host_link.init(1, 500000);
    info.init();

    menu.init();
}

void comms_mgr_t::update() {
    counter++;
    
    gcs_link.read_commands();
    host_link.read_commands();

    gcs_link.update();
    host_link.update();
    
    // 10hz human console output, (begins when gyros finish calibrating)
    if ( imu_node.getUInt("gyros_calibrated") != 2 ) {
        return;
    }

    if ( info_timer.update() ) {
        menu.update();
        if ( menu.display_pilot ) { info.write_pilot_in_ascii(); }
        if ( menu.display_gps ) { info.write_gps_ascii(); }
        if ( menu.display_airdata ) { info.write_airdata_ascii(); }
        if ( menu.display_imu ) { info.write_imu_ascii(); }
        if ( menu.display_nav ) { info.write_nav_ascii(); }
        if ( menu.display_nav_stats ) { info.write_nav_stats_ascii(); }
        if ( menu.display_act ) { info.write_actuator_out_ascii(); }
    }

    // 10 second heartbeat console output
    if ( heartbeat.update() ) {
        info.write_status_info_ascii();
        info.write_power_ascii();
        float elapsed_sec = (AP_HAL::millis() - tempTimer) / 1000.0;
        printf("Available mem: %d bytes\n",
               status_node.getUInt("available_memory"));
        printf("Performance = %.1f hz\n", counter/elapsed_sec);
        //PropertyNode("/").pretty_print();
        printf("\n");

#if 0
        // system info
        ExpandingString dma_info {};
        ExpandingString mem_info {};
        ExpandingString uart_info {};
        ExpandingString thread_info {};
        hal.util->dma_info(dma_info);
        hal.util->mem_info(mem_info);
        hal.util->uart_info(uart_info);
        hal.util->thread_info(thread_info);
        console->printf("dma info:\n%s\n", dma_info.get_string());
        console->printf("mem info:\n%s\n", mem_info.get_string());
        console->printf("uart info:\n%s\n", uart_info.get_string());
        // console->printf("thread info:\n%s\n", thread_info.get_string());
#endif
    }
}
