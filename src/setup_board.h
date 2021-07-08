#pragma once

#include <AP_HAL/AP_HAL.h>
#if HAL_OS_POSIX_IO
#include <stdio.h>
#endif
#include <AP_SerialManager/AP_SerialManager.h>

// work around these type errors:
//   error: 'vsnprintf' was not declared in this scope
#undef _GLIBCXX_USE_C99_STDIO

// Firmware rev (needs to be updated here manually to match release number)
const int FIRMWARE_REV = 500;

// Make thes available for convenience (they should be declared in the
// main source file):
// * hal.analog, hal.rcin, etc.
// * console->printf()
extern const AP_HAL::HAL& hal;
extern AP_HAL::UARTDriver *console;
extern AP_SerialManager serial_manager;

#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
extern AP_InertialSensor ins;
extern AP_AHRS_DCM ahrs;  // need ...
extern AP_Baro baro; // Compass tries to set magnetic model based on location.
extern Compass compass;

// it would be nice if these were dynamically detected, but they drive
// structure allocations and I need to think through that part of it
// carefully.
const uint8_t MAX_RCIN_CHANNELS = 16;
const uint8_t MAX_RCOUT_CHANNELS = 8;

// mRo Pixracer: can run the ekf at 100hz
#define AURA_ONBOARD_EKF

// this is the master loop update rate.
const int MASTER_HZ = 100;
const int DT_MILLIS = (1000 / MASTER_HZ);

// Please read the important notes in the source tree about Teensy
// baud rates vs. host baud rates.
const int DEFAULT_BAUD = 500000;

