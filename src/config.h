// configuration and eeprom

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "props2.h"

class config_t {
    
private:
    
    PropertyNode config_node;
    
 public:
    
    void init();               // load config.json from sd card
    
    uint16_t serial_number = 0;
    uint16_t read_serial_number();
    uint16_t set_serial_number(uint16_t value);

    bool load_json_config();
    
    // int read_storage();
    // int write_storage();

    void actuator_gain_defaults();
    //void force_config_aura3();
    //void force_config_goldy3();
    void reset_defaults();

};
