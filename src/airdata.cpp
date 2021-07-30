// Module to query air data sensors

#include "setup_board.h"
#include "airdata.h"

void airdata_t::setup() {
    airdata_node = PropertyNode("/sensors/airdata");
    error_count = 0;
    airdata_node.setUInt("error_count", error_count);

    console->printf("Initializing & calibrating airspeed...\n");
    // can't, private: airspeed.param[0].type = TYPE_I2C_MS5525;
    airspeed.init();
    airspeed.calibrate(false);
    
    console->printf("Initializing & calibrating barometer...\n");
    barometer.init();
    barometer.calibrate();
}

void airdata_t::update() {
    // assumes we are being called at 100hz
    airspeed.update(false);
    if ( airspeed.healthy() ) {
        airdata_node.setDouble("airspeed_mps", airspeed.get_airspeed());
        airdata_node.setDouble("diffPress_pa", airspeed.get_differential_pressure());
        float temperature;
        airspeed.get_temperature(temperature);
        airdata_node.setDouble("temp_C", temperature);
    }
    
    // collect readings @ 20hz
    barometer.accumulate();
    if ( counter++ >= 5 ) {
        counter = 0;
        barometer.update();

        if ( barometer.healthy() ) {
            airdata_node.setDouble("baro_press_pa", barometer.get_pressure());
            airdata_node.setDouble("baro_tempC", barometer.get_temperature());
        }
    }
    if ( !airspeed.healthy() or !barometer.healthy() ) {
        error_count++;
        airdata_node.setUInt("error_count", error_count);
    }
}

