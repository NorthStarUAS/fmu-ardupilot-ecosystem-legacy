#include "imu_mgr.h"

#include "setup_board.h"
#include "config.h"

#include <AP_HAL/AP_HAL.h>

// Setup imu defaults:
// Goldy3 has mpu9250 on SPI CS line 24
void imu_mgr_t::defaults_goldy3() {
    config.imu_cfg.interface = 0;       // SPI
    config.imu_cfg.pin_or_address = 24; // CS pin
    defaults_common();
}

// Setup imu defaults:
// Aura3 has mpu9250 on I2C Addr 0x68
void imu_mgr_t::defaults_aura3() {
    config.imu_cfg.interface = 1;       // i2c
    config.imu_cfg.pin_or_address = 0x68; // mpu9250 i2c addr
    defaults_common();
}

// Setup imu defaults:
// Aura3 has mpu9250 on I2C Addr 0x68
void imu_mgr_t::defaults_common() {
    Eigen::Matrix3f strapdown3x3 = Eigen::Matrix3f::Identity();
    for ( int i = 0; i < 9; i++ ) {
        // no need to worry about row vs. column major here (symmetrical ident)
        config.imu_cfg.strapdown_calib[i] = strapdown3x3.data()[i];
    }
    strapdown = Eigen::Matrix4f::Identity();

    for ( int i = 0; i < 3; i++ ) {
        config.imu_cfg.accel_scale[i] = 1.0;
    }
    for ( int i = 0; i < 3; i++ ) {
        config.imu_cfg.accel_translate[i] = 0.0;
    }
    accel_affine = Eigen::Matrix4f::Identity();
    
    mag_affine = Eigen::Matrix4f::Identity();
    for ( int i = 0; i < 16; i++ ) {
        // no need to worry about row vs. column major here (symmetrical ident)
        config.imu_cfg.mag_affine[i] = mag_affine.data()[i];
    }
}

// Update the R matrix (called after loading/receiving any new config message)
void imu_mgr_t::set_strapdown_calibration() {
    // config.imu_cfg.orientation is row major, but internally Eigen defaults
    // to column major.
    Eigen::Matrix3f strapdown3x3 = Eigen::Matrix<float, 3, 3, Eigen::RowMajor>(config.imu_cfg.strapdown_calib);
    strapdown = Eigen::Matrix4f::Identity();
    strapdown.block(0,0,3,3) = strapdown3x3;
    Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; i++ ) {
        scale(i,i) = config.imu_cfg.accel_scale[i];
    }
    Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
    for (int i = 0; i < 3; i++ ) {
        // column major
        translate(i,3) = config.imu_cfg.accel_translate[i];
    }
    accel_affine = translate * strapdown * scale;
    console->printf("Accel affine calibration matrix:\n");
    for ( int i = 0; i < 4; i++ ) {
        console->printf("  ");
        for ( int j = 0; j < 4; j++ ) {
            console->printf("%.4f ", accel_affine(i,j));
        }
        console->printf("\n");
    }
    console->printf("IMU strapdown calibration matrix:\n");
    for ( int i = 0; i < 4; i++ ) {
        console->printf("  ");
        for ( int j = 0; j < 4; j++ ) {
            console->printf("%.4f ", strapdown(i,j));
        }
        console->printf("\n");
    }
}

// setup accel temp calibration
//void imu_mgr_t::set_accel_calibration() {
    //ax_cal.init(config.imu_cfg.ax_coeff, config.imu_cfg.min_temp, config.imu_cfg.max_temp);
    //ay_cal.init(config.imu_cfg.ay_coeff, config.imu_cfg.min_temp, config.imu_cfg.max_temp);
    //az_cal.init(config.imu_cfg.az_coeff, config.imu_cfg.min_temp, config.imu_cfg.max_temp);
//}

// update the mag calibration matrix from the config structur
void imu_mgr_t::set_mag_calibration() {
    mag_affine = Eigen::Matrix4f::Identity();
    mag_affine = Eigen::Matrix<float, 4, 4, Eigen::RowMajor>(config.imu_cfg.mag_affine);
    
    console->printf("Magnetometer affine matrix:\n");
    for ( int i = 0; i < 4; i++ ) {
        console->printf("  ");
        for ( int j = 0; j < 4; j++ ) {
            console->printf("%.4f ", mag_affine(i,j));
        }
        console->printf("\n");
    }
}

// configure the IMU settings and setup the ISR to aquire the data
void imu_mgr_t::setup() {
    imu_hal.setup();
}

// query the imu and update the structures
void imu_mgr_t::update() {
    // static uint8_t accel_count = ins.get_accel_count();
    // static uint8_t gyro_count = ins.get_gyro_count();

    imu_hal.update();
    imu_millis = imu_hal.raw_millis;
    
    accels_raw << imu_hal.accel.x, imu_hal.accel.y, imu_hal.accel.z, 1.0;
    gyros_raw << imu_hal.gyro.x, imu_hal.gyro.y, imu_hal.gyro.z, 1.0;

    Eigen::Vector4f mags_precal;
    mags_precal << imu_hal.mag.x, imu_hal.mag.y, imu_hal.mag.z, 1.0;
    mags_raw = strapdown * mags_precal;
    
    accels_cal = accel_affine * accels_raw;
    gyros_cal = strapdown * gyros_raw;

    //accels_cal(0) = ax_cal.calibrate(accels_nocal(0), tempC);
    //accels_cal(1) = ay_cal.calibrate(accels_nocal(1), tempC);
    //accels_cal(2) = az_cal.calibrate(accels_nocal(2), tempC);
        
    mags_cal = mag_affine * mags_raw;
    
    if ( gyros_calibrated < 2 ) {
        calibrate_gyros();
    } else {
        gyros_cal.segment(0,3) -= gyro_startup_bias;
    }
}


// stay alive for up to 15 seconds looking for agreement between a 1
// second low pass filter and a 0.1 second low pass filter.  If these
// agree (close enough) for 4 consecutive seconds, then we calibrate
// with the 1 sec low pass filter value.  If time expires, the
// calibration fails and we run with raw gyro values.
void imu_mgr_t::calibrate_gyros() {
    if ( gyros_calibrated == 0 ) {
        console->printf("Initialize gyro calibration: ");
        slow = gyros_cal.segment(0,3);
        fast = gyros_cal.segment(0,3);
        total_timer = AP_HAL::millis();
        good_timer = AP_HAL::millis();
        output_timer = AP_HAL::millis();
        gyros_calibrated = 1;
    }

    fast = 0.95 * fast + 0.05 * gyros_cal.segment(0,3);
    slow = 0.995 * fast + 0.005 * gyros_cal.segment(0,3);
    // use 'slow' filter value for calibration while calibrating
    gyro_startup_bias << slow;

    float max = (slow - fast).cwiseAbs().maxCoeff();
    if ( max > cutoff ) {
        good_timer = AP_HAL::millis();
    }
    if (  AP_HAL::millis() - output_timer >= 1000 ) {
        output_timer = AP_HAL::millis();
        if ( AP_HAL::millis() - good_timer < 1000 ) {
            console->printf("x");
        } else {
            console->printf("*");
        }
    }
    if ( AP_HAL::millis() - good_timer > 4100 || AP_HAL::millis() - total_timer > 15000 ) {
        console->printf("\n");
        // set gyro zero points from the 'slow' filter.
        gyro_startup_bias = slow;
        gyros_calibrated = 2;
        // update(); // update imu_calib values before anything else get's a chance to read them // FIXME???
        console->printf("Average gyro startup bias: %.4f %.4f %.4f\n",
                        gyro_startup_bias(0), gyro_startup_bias(1),
                        gyro_startup_bias(2));
        if ( AP_HAL::millis() - total_timer > 15000 ) {
            console->printf("gyro init: too much motion, using best average guess.\n");
        } else {
            console->printf("gyro init: success.\n");
        }
    }
}

// global shared instance
imu_mgr_t imu_mgr;
