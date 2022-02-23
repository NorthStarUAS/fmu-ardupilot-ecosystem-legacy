#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_ExternalAHRS/AP_ExternalAHRS.h>
#include <AP_IOMCU/AP_IOMCU.h>
#include <GCS_MAVLink/GCS_Dummy.h>

#include "setup_board.h"

#include "comms/comms_mgr.h"
#include "config.h"
#include "guidance/guidance_mgr.h"
#include "led.h"
#include "nav/nav_mgr.h"
#include "props2.h"
#include "sensors/airdata.h"
#include "sensors/gps_mgr.h"
#include "sensors/imu_mgr.h"
#include "sensors/pilot.h"
#include "sensors/power.h"
#include "sim/sim_mgr.h"
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

static comms_mgr_t comms_mgr;
static config_t config;
static airdata_t airdata;
static gps_mgr_t gps_mgr;
static guidance_mgr_t guidance_mgr;
static led_t led;
static power_t power;
static sim_mgr_t sim;
static state_mgr_t state_mgr;

static gimbal_mavlink_t gimbal;

static RateLimiter maintimer(MASTER_HZ);

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

    // before pilot (before soft_armed, so stat() will work)
    sim.init();
    sim.reset();
    
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

    // guidance
    guidance_mgr.init();
    
    // led status
    led.init();

    // ekf init (just prints availability status)
    nav_mgr.init();

    // additional derived/computed/estimated values
    state_mgr.init();
     
    // do these after gps initialization
    comms_mgr.init();

    gimbal.init();

    printf("Setup finished.\n");
    printf("Ready and transmitting...\n");
}

// main loop
void loop() {
    const float dt = 1.0 / MASTER_HZ;
    
    if ( maintimer.update() ) {
        status_node.setUInt("main_loop_timer_misses", maintimer.misses);
        
        // 1. Sense motion
        imu_mgr.update();

        // 2. Check for gps updates
        gps_mgr.update();

        sim.update();
        
        // 3. Estimate location and attitude
        if ( config_nav_node.getString("selected") != "none" ) {
            nav_mgr.update();
        }

        state_mgr.update(1000.0 / MASTER_HZ);

        // airdata
        airdata.update();

        // read power values
        power.update();

        // guidance
        guidance_mgr.update(dt);
        
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

        // comms
        comms_mgr.update();
    }
}

AP_HAL_MAIN();
