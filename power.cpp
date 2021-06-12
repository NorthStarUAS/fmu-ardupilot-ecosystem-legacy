#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor_Analog.h>

#include "setup_board.h"

#include "config.h"

#include "power.h"

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// fixme/configurable
static float amps_offset = 0.0;

void power_t::setup() {
    console->printf("Battery volt pin: %d\n", AP_BATT_VOLT_PIN);
    console->printf("Battery current pin: %d\n", AP_BATT_CURR_PIN);
    console->printf("Volt divider: %.2f\n", AP_BATT_VOLTDIVIDER_DEFAULT);
    console->printf("Amp per volt: %.2f\n", AP_BATT_CURR_AMP_PERVOLT_DEFAULT);
    _volt_pin_analog_source = hal.analogin->channel(AP_BATT_VOLT_PIN);
    _curr_pin_analog_source = hal.analogin->channel(AP_BATT_CURR_PIN);
}

void power_t::update() {
    // avionics voltage
    avionics_v = hal.analogin->board_voltage();
    
    // battery volts / amps
    battery_volts = _volt_pin_analog_source->voltage_average() * AP_BATT_VOLTDIVIDER_DEFAULT;
    battery_amps = (_curr_pin_analog_source->voltage_average() - amps_offset) * AP_BATT_CURR_AMP_PERVOLT_DEFAULT;
}

// shared global instance
power_t power;
