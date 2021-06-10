#include "setup_board.h"

#include "airdata.h"
#include "comms.h"
#include "config.h"
#include "imu_mgr.h"
#include "led.h"
#include "mixer.h"
#include "power.h"

// starting point for writing big eeprom struct
static const int CONFIG_OFFSET = 2;

static const uint8_t START_OF_CFG0 = 147;
static const uint8_t START_OF_CFG1 = 224;

// global definitions
uint16_t serial_number;

config_t::config_t() {
    const AP_HAL::HAL& hal = AP_HAL::get_HAL();
    storage = hal.storage;
}

uint16_t config_t::read_serial_number() {
    uint8_t buf[2];
    storage->read_block(buf, 0, 2);
    // Serial.printf(" raw serial number read %d %d\n", hi, lo);
    serial_number = *(uint16_t *)(&buf);
    return serial_number;
};

uint16_t config_t::set_serial_number(uint16_t value) {
    serial_number = value;
    uint8_t *buf;
    buf = (uint8_t *)(&value);
    // Serial.printf(" set serial number raw: %d %d\n", hi, lo);
    storage->write_block(0, buf, 2);
    return serial_number;
};

static int mycopy(uint8_t *src, uint8_t *dst, int len) {
    for ( int i = 0; i < len; i++ ) {
        dst[i] = src[i];
    }
    return len;
}

int config_t::read_storage() {
    // call pack to initialize internal stucture len's
    airdata_cfg.pack(); 
    board_cfg.pack();
    ekf_cfg.pack();
    imu_cfg.pack();
    mixer_matrix_cfg.pack();
    power_cfg.pack();
    pwm_cfg.pack();
    stab_cfg.pack();
    
    packed_config_t packed;
    uint8_t *buf = (uint8_t *)&packed;
    int status = 0;
    if ( sizeof(packed) + CONFIG_OFFSET <= HAL_STORAGE_SIZE - 2 /* checksum */ + 1 ) {
        console->printf("Loading EEPROM.  Size: %d\n", sizeof(packed));
        storage->read_block(buf, CONFIG_OFFSET, sizeof(packed));
        console->printf("Finished reading EEPROM\n");
        uint8_t chksum_buf[2];
        storage->read_block(&chksum_buf, CONFIG_OFFSET + sizeof(packed), 2);
        uint8_t read_cksum0 = chksum_buf[0];
        uint8_t read_cksum1 = chksum_buf[1];
        console->printf("Read checksum: %d %d\n", read_cksum0, read_cksum1);
        uint8_t calc_cksum0 = 0;
        uint8_t calc_cksum1 = 0;
        comms.serial.checksum( START_OF_CFG0 /* arbitrary magic # */, START_OF_CFG1 /* arbitrary magic # */, buf, sizeof(packed), &calc_cksum0, &calc_cksum1 );
        console->printf("Compute checksum to compare to stored ...\n");
        if ( read_cksum0 != calc_cksum0 || read_cksum1 != calc_cksum1 ) {
            console->printf("Check sum error!\n");
        } else {
            console->printf("Unpacking stored structures...\n");
            status = 1;
            // unpack the config structures from the load buffer.
            airdata_cfg.unpack((uint8_t *)&packed.airdata, airdata_cfg.len);
            board_cfg.unpack((uint8_t *)&packed.board, board_cfg.len);
            ekf_cfg.unpack((uint8_t *)&packed.ekf, ekf_cfg.len);
            imu_cfg.unpack((uint8_t *)&packed.imu, imu_cfg.len);
            mixer_matrix_cfg.unpack((uint8_t *)&packed.mixer_matrix, mixer_matrix_cfg.len);
            power_cfg.unpack((uint8_t *)&packed.power, power_cfg.len);
            pwm_cfg.unpack((uint8_t *)&packed.pwm, pwm_cfg.len);
            stab_cfg.unpack((uint8_t *)&packed.stab, stab_cfg.len);
        }
    } else {
        console->printf("ERROR: config structure too large for EEPROM hardware!\n");
    }
    return status;
}

int config_t::write_storage() {
    // note: storage->write_block() writes to a block of memoory.  There is an
    // AP_HAL thread that then copies any changed bytes to physical eeprom.
    
    // create packed version of messages
    airdata_cfg.pack();
    board_cfg.pack();
    ekf_cfg.pack();
    imu_cfg.pack();
    mixer_matrix_cfg.pack();
    power_cfg.pack();
    pwm_cfg.pack();
    stab_cfg.pack();

    // copy the packed config structures to the save buffer
    packed_config_t packed;
    mycopy(airdata_cfg.payload, (uint8_t *)&packed.airdata, airdata_cfg.len); 
    mycopy(board_cfg.payload, (uint8_t *)&packed.board, board_cfg.len);
    mycopy(ekf_cfg.payload, (uint8_t *)&packed.ekf, ekf_cfg.len);
    mycopy(imu_cfg.payload, (uint8_t *)&packed.imu, imu_cfg.len);
    mycopy(mixer_matrix_cfg.payload, (uint8_t *)&packed.mixer_matrix, mixer_matrix_cfg.len);
    mycopy(power_cfg.payload, (uint8_t *)&packed.power, power_cfg.len);
    mycopy(pwm_cfg.payload, (uint8_t *)&packed.pwm, pwm_cfg.len);
    mycopy(stab_cfg.payload, (uint8_t *)&packed.stab, stab_cfg.len);
    
    console->printf("Write EEPROM (any changed bytes.)  Size: %d\n",
                    sizeof(packed));
    int status = 0;
    if ( sizeof(packed) + CONFIG_OFFSET <= HAL_STORAGE_SIZE - 2 /* checksum */ + 1 ) {
        uint8_t calc_cksum0 = 0;
        uint8_t calc_cksum1 = 0;
        comms.serial.checksum( START_OF_CFG0 /* arbitrary magic # */, START_OF_CFG1 /* arbitrary magic # */, (uint8_t *)&packed, sizeof(packed), &calc_cksum0, &calc_cksum1 );
        storage->write_block(CONFIG_OFFSET, &packed, sizeof(packed));
        storage->write_block(CONFIG_OFFSET+sizeof(packed), &calc_cksum0, 1);
        storage->write_block(CONFIG_OFFSET+sizeof(packed)+1, &calc_cksum1, 1);
        status = 1;
    } else {
        console->printf("ERROR: config structure too large for EEPROM hardware!\n");
    }
    return status;
}

// force/hard-code a specific board config if desired
void config_t::force_config_aura3() {
    console->printf("Forcing an aura v2 config\n");
    config.board_cfg.board = 1;    // 0 = marmot v1, 1 = aura v2
    imu_mgr.defaults_aura3();
    airdata.defaults_aura3();
    led.defaults_aura3();
    config.power_cfg.have_attopilot = true;
    pwm.act_gain_defaults();
    mixer.setup();
    config.stab_cfg.sas_rollaxis = true;
    config.stab_cfg.sas_pitchaxis = true;
    config.stab_cfg.sas_yawaxis = true;
    config.stab_cfg.sas_rollgain = 0.2;
    config.stab_cfg.sas_pitchgain = 0.2;
    config.stab_cfg.sas_yawgain = 0.2;
    config.ekf_cfg.select = message::enum_nav::none;
    config.write_storage();
}

// force/hard-code a specific board config if desired
void config_t::force_config_goldy3() {
    console->printf("Forcing a bfs/marmot config\n");
    config.board_cfg.board = 0;    // 0 = marmot v1, 1 = aura v2
    imu_mgr.defaults_goldy3();
    airdata.defaults_goldy3();
    led.defaults_goldy3();
    pwm.act_gain_defaults();
    mixer.setup();
    config.stab_cfg.sas_rollaxis = true;
    config.stab_cfg.sas_pitchaxis = true;
    config.stab_cfg.sas_yawaxis = true;
    config.stab_cfg.sas_rollgain = 0.2;
    config.stab_cfg.sas_pitchgain = 0.2;
    config.stab_cfg.sas_yawgain = 0.2;
    config.ekf_cfg.select = message::enum_nav::none;
    config.write_storage();
}

void config_t::reset_defaults() {
    console->printf("Setting default config ...\n");
    config.board_cfg.board = 0;
    imu_mgr.defaults_goldy3();
    led.defaults_goldy3();
    pwm.act_gain_defaults();
    mixer.sas_defaults();
    mixer.setup();
    config.power_cfg.have_attopilot = false;
}

// global shared instance
config_t config;
