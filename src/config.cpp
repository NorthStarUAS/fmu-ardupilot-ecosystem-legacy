#include "setup_board.h"

#include "config.h"
#include "sensors/imu_mgr.h"
#include "sensors/pilot.h"              // reset defaults

// starting point for writing big eeprom struct
static const int CONFIG_OFFSET = 2;

static const uint8_t START_OF_CFG0 = 147;
static const uint8_t START_OF_CFG1 = 224;

uint16_t config_t::read_serial_number() {
    // uint8_t buf[2];
    // uint16_t buf;
    hal.storage->read_block(&serial_number, 0, 2);
    // Serial.printf(" raw serial number read %d %d\n", hi, lo);
    // serial_number = *(uint16_t *)(&buf);
    // serial_number = *buf;
    if ( !config_node.isNull() ) {
        config_node.setInt("serial_number", serial_number);
    }
    return serial_number;
};

uint16_t config_t::set_serial_number(uint16_t value) {
    serial_number = value;
    uint8_t *buf;
    buf = (uint8_t *)(&value);
    // Serial.printf(" set serial number raw: %d %d\n", hi, lo);
    hal.storage->write_block(0, buf, 2);
    return serial_number;
};

void config_t::reset_defaults() {
    console->printf("Setting default config ...\n");
    imu_mgr.defaults();
    pilot.mixer.init();
    pilot.mixer.sas_defaults();
}

void config_t::init() {
    config_node = PropertyNode("/config");
}

bool config_t::load_json_config() {
    const char *file_path = "config.json";
    if ( !config_node.load(file_path) ) {
        console->printf("Config file loading failed: %s\n", file_path);
        return false;
    }
    return true;
}
