// configuration and eeprom

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "props2.h"
#include "aura4_messages.h"

class config_t {
    
private:
    
    AP_HAL::Storage *storage = NULL;
    
    int config_size = 0;
    struct packed_config_t {
        message::config_airdata_t::_compact_t airdata;
        message::config_board_t::_compact_t board;
        message::config_ekf_t::_compact_t ekf;
        message::config_imu_t::_compact_t imu;
        message::config_mixer_matrix_t::_compact_t mixer_matrix;
        message::config_power_t::_compact_t power;
        message::config_pwm_t::_compact_t pwm;
        message::config_stability_damping_t::_compact_t stab;
    };

    PropertyNode config_node;
    
 public:
    
    config_t();
    
    message::config_airdata_t airdata_cfg;
    message::config_board_t board_cfg;
    message::config_ekf_t ekf_cfg;
    message::config_imu_t imu_cfg;
    message::config_mixer_matrix_t mixer_matrix_cfg;
    message::config_power_t power_cfg;
    message::config_pwm_t pwm_cfg;
    message::config_stability_damping_t stab_cfg;

    uint16_t serial_number = 0;
    uint16_t read_serial_number();
    uint16_t set_serial_number(uint16_t value);
    int read_storage();
    int write_storage();

    void actuator_gain_defaults();
    void force_config_aura3();
    void force_config_goldy3();
    void reset_defaults();

    void setup();               // load config.json from sd card
};

extern config_t config;
