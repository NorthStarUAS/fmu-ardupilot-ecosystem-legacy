#include <AP_HAL/AP_HAL.h>
#include <stdio.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>

#include "setup_board.h"
#include "imu_hal.h"

// static AP_InertialSensor ins;
// static AP_AHRS_DCM ahrs;  // need ...
// static AP_Baro baro; // Compass tries to set magnetic model based on location.
// static Compass compass;

// initialize the imu sensor(s)
void imu_hal_t::init() {
    printf("AP_InertialSensor startup...\n");
    hal.scheduler->delay(100);
    ins.init(100);
    printf("Number of detected accels : %u\n", ins.get_accel_count());
    printf("Number of detected gyros  : %u\n", ins.get_gyro_count());
    printf("ahrs.init()\n");
    ahrs.init();
    printf("compass.init()\n");
    compass.init();
    printf("Number of detected compasses  : %u\n", compass.get_count());
    hal.scheduler->delay(100);
}

// query the imu
void imu_hal_t::update() {
    // static uint8_t accel_count = ins.get_accel_count();
    // static uint8_t gyro_count = ins.get_gyro_count();
 
    raw_millis = AP_HAL::millis();

    ins.wait_for_sample();      // wait until we have a sample
    ins.update();               // read

    // for now just go with the 0'th INS sensor
    accel = ins.get_accel(0);
    gyro = ins.get_gyro(0);
    temp_C = ins.get_temperature(0);

    compass.read();
    mag = compass.get_field(0);
}
