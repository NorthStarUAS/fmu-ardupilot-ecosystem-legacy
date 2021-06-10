#pragma once

#include <AP_HAL/AP_HAL.h>

// Firmware rev (needs to be updated here manually to match release number)
const int FIRMWARE_REV = 500;

extern AP_HAL::UARTDriver *console;

// automatic configuration
#if defined(ARDUINO_TEENSY32) || defined(ARDUINO_TEENSY40)
 #define AURA_V2
#elif defined(ARDUINO_TEENSY36)
 #define MARMOT_V1
#endif

// mRo Pixracer: can run the ekf at 100hz
// teensy32 can run the ekf at 50hz
#if defined(ARDUINO_TEENSY32)
#undef AURA_ONBOARD_EKF
#else
#define AURA_ONBOARD_EKF
#endif

// this is the master loop update rate.
const int MASTER_HZ = 100;
const int DT_MILLIS = (1000 / MASTER_HZ);

// Please read the important notes in the source tree about Teensy
// baud rates vs. host baud rates.
const int DEFAULT_BAUD = 500000;

extern uint16_t serial_number;

