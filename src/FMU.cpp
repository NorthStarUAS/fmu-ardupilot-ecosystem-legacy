#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_IOMCU/AP_IOMCU.h>
#include <GCS_MAVLink/GCS_Dummy.h>

#include "setup_board.h"

#include "comms/info.h"
#include "comms/rc_link.h"
#include "config.h"
#include "led.h"
#include "menu.h"
#include "nav_mgr.h"
#include "props2.h"
#include "sensors/airdata.h"
#include "sensors/gps_mgr.h"
#include "sensors/imu_mgr.h"
#include "sensors/pilot.h"
#include "sensors/power.h"
#include "state/state_mgr.h"
#include "util/ratelimiter.h"

#include "gimbal/gimbal_mavlink.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static AP_BoardConfig BoardConfig;
AP_HAL::UARTDriver *console;
// AP_HAL::UARTDriver *console = hal.serial(1); // telemetry 1

// Serial manager is needed for UART communications
AP_SerialManager serial_manager;

#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
AP_InertialSensor ins;
AP_AHRS ahrs;  // need ...
AP_Baro baro; // Compass tries to set magnetic model based on location.
Compass compass;

// needed by imu_hal and airdata(baro)
#if HAL_EXTERNAL_AHRS_ENABLED
static AP_ExternalAHRS eAHRS;
#endif // HAL_EXTERNAL_AHRS_ENABLED

// create fake gcs object (needed for the AP_HAL baro and gps drivers)
GCS_Dummy _gcs;
const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
SITL::SITL sitl;
#endif

static PropertyNode config_node;
static PropertyNode config_nav_node;
static PropertyNode pilot_node;
static PropertyNode status_node;

static config_t config;
static airdata_t airdata;
static gps_mgr_t gps_mgr;
static rc_link_t gcs_link;
static info_t info;
static led_t led;
static menu_t menu;
static power_t power;
static state_mgr_t state_mgr;

static gimbal_mavlink_t gimbal;

static RateLimiter maintimer(MASTER_HZ);
static RateLimiter heartbeat(0.1);
static RateLimiter debugging(10);

// -Wmissing-declarations requires these
void setup();
void loop();

void setup() {
    BoardConfig.init();         // setup any board specific drivers
    serial_manager.init();

    console = hal.console;
    console->begin(57600);
    hal.scheduler->delay(2000); // give the electrons a chance to settle
    
    console->printf("\nRice Creek UAS FMU: Rev %d\n", FIRMWARE_REV);
    console->printf("You are seeing this message on the console interface.\n");
    console->printf("Host communication is on Serial1 @ %d baud (N81) no flow control.\n", HOST_BAUD);

    // load config from sd card
    config.init();
    if ( !config.load_json_config() ) {
        config.reset_defaults();
    }
    
    // The following code (when enabled) will force setting a specific
    // device serial number when the device boots:
    if ( false ) {
        config.set_serial_number(117);
    }
    config.read_serial_number();
    console->printf("Serial Number: %d\n", config.read_serial_number());
    hal.scheduler->delay(100);
    
    // after config.init()
    config_node = PropertyNode("/config");
    config_nav_node = PropertyNode("/config/nav");
    pilot_node = PropertyNode("/pilot");
    status_node = PropertyNode("/status");
    
    status_node.setUInt("firmware_rev", FIRMWARE_REV);
    status_node.setUInt("master_hz", MASTER_HZ);
    status_node.setUInt("baud", TELEMETRY_BAUD);
    status_node.setUInt("serial_number", config_node.getUInt("serial_number"));
    
    // airdata
    airdata.init();
    
    // initialize the IMU and calibration matrices
    console->printf("before imu_mgr.init()\n");
    imu_mgr.init();
    imu_mgr.set_strapdown_calibration();
    imu_mgr.set_accel_calibration();
    imu_mgr.set_mag_calibration();
    
    // initialize the pilot interface (RC in, out & mixer)
    pilot.init();

    // initialize the gps receiver
    gps_mgr.init();

    // power sensing
    power.init();
    
    // led status
    led.init();

    // ekf init (just prints availability status)
    nav_mgr.init();

    // additional derived/computed/estimated values
    state_mgr.init();
     
    // do these after gps initialization
    gcs_link.init(2, 115200 /*FIXME 57600*/);
    info.init();

    menu.init();

    gimbal.init();
    
    printf("Setup finished.\n");
    printf("Ready and transmitting...\n");
}

// main loop
void loop() {
    if ( maintimer.update() ) {
        static uint32_t tempTimer = AP_HAL::millis();
        static uint32_t counter = 0;
        status_node.setUInt("main_loop_timer_misses", maintimer.misses);
        
        // 1. Sense motion
        imu_mgr.update();

        // 2. Check for gps updates
        gps_mgr.update();

        // 3. Estimate location and attitude
        if ( config_nav_node.getString("selected") != "none" ) {
            nav_mgr.update();
        }

        state_mgr.update(1000.0 / MASTER_HZ);
        
        // 10hz human console output, (begins when gyros finish calibrating)
        if ( debugging.update() ) {
            if ( imu_mgr.gyros_calibrated == 2 ) {
                menu.update();
                if ( menu.display_pilot ) { info.write_pilot_in_ascii(); }
                if ( menu.display_gps ) { info.write_gps_ascii(); }
                if ( menu.display_airdata ) { info.write_airdata_ascii(); }
                if ( menu.display_imu ) { info.write_imu_ascii(); }
                if ( menu.display_nav ) { info.write_nav_ascii(); }
                if ( menu.display_nav_stats ) { info.write_nav_stats_ascii(); }
                if ( menu.display_act ) { info.write_actuator_out_ascii(); }
            }
        }

        // 10 second heartbeat console output
        if ( heartbeat.update() ) {
            if ( imu_mgr.gyros_calibrated == 2 ) {
                info.write_status_info_ascii();
                info.write_power_ascii();
                float elapsed_sec = (AP_HAL::millis() - tempTimer) / 1000.0;
                console->printf("Available mem: %d bytes\n",
                                status_node.getUInt("available_memory"));
                console->printf("Performance = %.1f hz\n", counter/elapsed_sec);
                //PropertyNode("/").pretty_print();
                console->printf("\n");

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
        
        // airdata
        airdata.update();

        // read power values
        power.update();

        if ( pilot.read() ) {
            bool ap_state = pilot_node.getBool("ap_enabled");
            static bool last_ap_state = ap_state;
            if ( ap_state and !last_ap_state ) {
                console->printf("ap enabled\n");
            } else if ( !ap_state and last_ap_state ) {
                console->printf("ap disabled (manaul flight)\n");
            }
            last_ap_state = ap_state;
        }

        pilot.write();

        gimbal.update();
        
        // blink the led
        led.do_policy(imu_mgr.gyros_calibrated);
        led.update();

        // status
        status_node.setUInt("available_memory", hal.util->available_memory());

        gcs_link.read_commands();
        gcs_link.update();
        
    
        counter++;
    }
}

AP_HAL_MAIN();
