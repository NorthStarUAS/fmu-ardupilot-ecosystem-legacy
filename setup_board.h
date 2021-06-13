#pragma once

// FIXME: some of this still needs some cleanup in the context of AP / Chibios

#include <AP_HAL/AP_HAL.h>

// Firmware rev (needs to be updated here manually to match release number)
const int FIRMWARE_REV = 500;

// Make this available for notational convenience: console->printf()
extern AP_HAL::UARTDriver *console;

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

extern uint16_t serial_number;

