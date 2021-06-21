#include <AP_HAL/AP_HAL.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

// test
#include <AP_Common/ExpandingString.h>

#include "setup_board.h"

// #include "airdata.h"
#include "comms.h"
#include "config.h"
#include "gps_mgr.h"
#include "imu_mgr.h"
#include "led.h"
#include "nav_mgr.h"
#include "pilot.h"
#include "power.h"
#include "props2.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();
static AP_BoardConfig BoardConfig;
AP_HAL::UARTDriver *console = hal.console;
// AP_HAL::UARTDriver *console = hal.serial(1); // telemetry 1

// -Wmissing-declarations requires these
void setup();
void loop();

void setup() {
    BoardConfig.init();         // setup any board specific drivers

    console->begin(57600);
    hal.scheduler->delay(2000); // give the electrons a second to settle
    
    console->printf("\nRice Creek UAS FMU: Rev %d\n", FIRMWARE_REV);
    console->printf("You are seeing this message on the console interface.\n");
    console->printf("Sensor/config communication is on Serial1 @ %d baud (N81) no flow control.\n", DEFAULT_BAUD);
    
    // The following code (when enabled) will force setting a specific
    // device serial number when the device boots:
    if ( false ) {
        config.set_serial_number(117);
    }
    config.read_serial_number();
    console->printf("Serial Number: %d\n", config.read_serial_number());
    hal.scheduler->delay(100);

    config.setup();             // load config from sd card
    
    if ( !config.read_storage() ) {
        console->printf("Resetting eeprom to default values.");
        config.reset_defaults();
        config.write_storage();
    } else {
        console->printf("Successfully loaded config from eeprom storage.\n");
    }
    
    // update imu strapdown and mag_affine matrices from config
    imu_mgr.set_strapdown_calibration();
    imu_mgr.set_mag_calibration();
    
    // initialize the IMU
    imu_mgr.setup();

    // initialize the pilot interface (RC in, out & mixer)
    pilot.setup();

    // initialize the gps receiver
    gps_mgr.setup();

    // initialize air data (marmot v1)
    // airdata.setup();
    
    // power sensing
    power.setup();
    
    // led status
    led.setup();

    // ekf init (just prints availability status)
    nav_mgr.setup();

    comms.setup();              // do this after gps initialization

    console->printf("Setup finished.\n");
    console->printf("Ready and transmitting...\n");
}

// main loop
void loop() {
    static uint32_t mainTimer = AP_HAL::millis();
    static uint32_t hbTimer = AP_HAL::millis();
    static uint32_t debugTimer = AP_HAL::millis();

    static uint32_t tempTimer = AP_HAL::millis();
    static uint32_t counter = 0;

    // this is the heartbeat of the system here (DT_MILLIS)
    if ( AP_HAL::millis() - mainTimer >= DT_MILLIS ) {
        if ( AP_HAL::millis() - mainTimer > DT_MILLIS ) {
            comms.main_loop_timer_misses++;
            mainTimer = AP_HAL::millis(); // catch up
            if ( comms.main_loop_timer_misses % 25 == 0 ) {
                console->printf("WARNING: main loop is not completing on time!\n");
            }
        } else {
            mainTimer += DT_MILLIS;
        }
        counter++;
        
        // top priority, used for timing sync downstream.
        imu_mgr.update();

        if ( config.ekf_cfg.select != message::enum_nav::none ) {
            nav_mgr.update();
        }

        if ( true) {
            comms.output_counter += comms.write_pilot_in_bin();
            comms.output_counter += comms.write_gps_bin();
            // comms.output_counter += comms.write_airdata_bin();
            comms.output_counter += comms.write_power_bin();
            // do a little extra dance with the return value because
            // write_status_info_bin() can reset comms.output_counter (but
            // that gets ignored if we do the math in one step)
            uint8_t result = comms.write_status_info_bin();
            comms.output_counter += result;
            if ( config.ekf_cfg.select != message::enum_nav::none ) {
                comms.output_counter += comms.write_nav_bin();
            }
            // write imu message last: used as an implicit end of data
            // frame marker.
            comms.output_counter += comms.write_imu_bin();
            hal.scheduler->delay(1);
        }

        // 10 second heartbeat console output
        if ( AP_HAL::millis() - hbTimer >= 10000 ) {
            hbTimer = AP_HAL::millis();
            if ( imu_mgr.gyros_calibrated == 2 ) {
                comms.write_status_info_ascii();
                comms.write_power_ascii();
                float elapsed_sec = (AP_HAL::millis() - tempTimer) / 1000.0;
                console->printf("Available mem: %ld bytes\n",
                                hal.util->available_memory());
                console->printf("Performace = %.1f hz\n", counter / elapsed_sec);
                PropertyNode("/").pretty_print();
                console->printf("\n");

                if ( false ) {
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
                }
            }
        }
        
        // 10hz human console output, (begins when gyros finish calibrating)
        if ( AP_HAL::millis() - debugTimer >= 100 ) {
            debugTimer = AP_HAL::millis();
            if ( imu_mgr.gyros_calibrated == 2 ) {
                // comms.write_pilot_in_ascii();
                // comms.write_actuator_out_ascii();
                // comms.write_gps_ascii();
                // if ( config.ekf_cfg.select != message::enum_nav::none ) {
                //     comms.write_nav_ascii();
                // }
                // comms.write_airdata_ascii();
                // comms.write_status_info_ascii();
                // comms.write_imu_ascii();
            }
        }

        // poll the pressure sensors
        // airdata.update();

        // read power values
        power.update();

        // suck in any available gps messages
        gps_mgr.update();

        if ( pilot.read() ) {
            static bool last_ap_state = pilot.ap_enabled();
            if ( pilot.ap_enabled() and !last_ap_state ) {
                console->printf("ap enabled\n");
            } else if ( !pilot.ap_enabled() and last_ap_state ) {
                console->printf("ap disabled (manaul flight)\n");
            }
            // pwm.update();
            last_ap_state = pilot.ap_enabled();
        }

        // read in any host commmands (config, inceptors, etc.)
        comms.read_commands();

        if ( pilot.changed ) {
            pilot.write();
        }

        // blink the led
        led.do_policy(imu_mgr.gyros_calibrated, gps_mgr.gps);
        led.update();
    }
}

AP_HAL_MAIN();
