// Module to query air data sensors

#include "airdata.h"

void airdata_t::setup() {
    airdata_node = PropertyNode("/sensors/airdata");
    error_count = 0;
    airdata_node.setUInt("error_count", error_count);
    // initialize the barometer
    barometer.init();
    barometer.calibrate();
}

void airdata_t::update() {
    // assumes we are being called at 100hz
    barometer.accumulate();

    // collect readings @ 20hz
    if ( counter++ >= 5 ) {
        counter = 0;
        barometer.update();

        if ( barometer.healthy() ) {
            airdata_node.setFloat("baro_press_pa", barometer.get_pressure());
            airdata_node.setFloat("baro_tempC", barometer.get_temperature());
        } else {
            error_count++;
            airdata_node.setUInt("error_count", error_count);
        }
    }
    
#if 0
    bool result;

    // read barometer (static pressure sensor)
    if ( config.airdata.barometer == 0 || config.airdata.barometer == 1 ) {
        if ( bme280_status ) {
            bme280.getData(&baro_press, &baro_temp, &baro_hum);
        }
    } else if ( config.airdata.barometer == 2 ) {
        if ( ams_barometer.getData(&baro_press, &baro_temp) ) {
            ams_baro_found = true;
        } else {
            if ( ams_baro_found ) {
                // Serial.println("Error while reading sPress sensor.");
                error_count++;
            }
        }
    } else if ( config.airdata.barometer == 3 ) {
        // BMP180 (requires a delicate dance of requesting a read,
        // then coming back some amount of millis later to do the
        // actual read.)
        static int bmp180_state = 0;
        static unsigned long wait_until;
        double tmp_temp;
        if ( bmp180_status ) {
            if ( bmp180_state == 0 ) {
                wait_until = bmp180.startTemperature() + millis();
                bmp180_state += 1;
            } else if ( bmp180_state == 1 ) {
                if ( millis() > wait_until ) {
                    if ( bmp180.getTemperature(tmp_temp) ) {
                        baro_temp = tmp_temp;
                    }
                    bmp180_state += 1;
                }
            } else if ( bmp180_state == 2 ) {
                wait_until = bmp180.startPressure(2) + millis();
                bmp180_state += 1;
            } else if ( bmp180_state == 3 ) {
                if ( millis() > wait_until ) {
                    double tmp;
                    if ( bmp180.getPressure(tmp, tmp_temp) ) {
                        baro_press = tmp;
                    }
                    bmp180_state = 0;
                }
            }
        }
    }

    if ( config.airdata.pitot == 0 ) {
        result = ms45_pitot.getData(&diffPress_pa, &temp_C);
    } else if ( config.airdata.pitot == 1 ) {
        result = ms55_pitot.getData(&diffPress_pa, &temp_C);
    } else if ( config.airdata.pitot == 2 ) {
        result = ams_pitot.getData(&diffPress_pa, &temp_C);
    } else {
        result = false;
    }
    if ( !result ) {
        if ( pitot_found ) {
            // Serial.println("Error while reading pitot sensor.");
            error_count++;
        }
    } else {
        pitot_found = true;
    }
#endif
}

