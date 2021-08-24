#include "imu_mgr.h"

#include "setup_board.h"

#include <AP_HAL/AP_HAL.h>

// Setup imu defaults:
void imu_mgr_t::defaults() {
    strapdown = Eigen::Matrix3f::Identity();
    for ( int i = 0; i < 9; i++ ) {
        imu_calib_node.setDouble("strapdown", strapdown.data()[i], i);
    }

    accel_affine = Eigen::Matrix4f::Identity();
    for ( int i = 0; i < 16; i++ ) {
        imu_calib_node.setDouble("accel_affine", accel_affine.data()[i], i);
    }
    
    mag_affine = Eigen::Matrix4f::Identity();
    for ( int i = 0; i < 16; i++ ) {
        imu_calib_node.setDouble("mag_affine", mag_affine.data()[i], i);
    }
}

// Update the R matrix (called after loading/receiving any new config message)
void imu_mgr_t::set_strapdown_calibration() {
    strapdown = Eigen::Matrix3f::Identity();
    for ( int i = 0; i < 3; i++ ) {
        for ( int j = 0; j < 3; j++ ) {
            strapdown(i,j) = imu_calib_node.getDouble("strapdown", i*3+j);
        }
    }
    
    console->printf("IMU strapdown calibration matrix:\n");
    for ( int i = 0; i < 3; i++ ) {
        console->printf("  ");
        for ( int j = 0; j < 3; j++ ) {
            console->printf("%.4f ", strapdown(i,j));
        }
        console->printf("\n");
    }
}

// update the mag calibration matrix from the config structur
void imu_mgr_t::set_accel_calibration() {
    accel_affine = Eigen::Matrix4f::Identity();
    for ( int i = 0; i < 4; i++ ) {
        for ( int j = 0; j < 4; j++ ) {
            accel_affine(i,j) = imu_calib_node.getDouble("accel_affine", i*4+j);
        }
    }

    console->printf("Accelerometer affine matrix:\n");
    for ( int i = 0; i < 4; i++ ) {
        console->printf("  ");
        for ( int j = 0; j < 4; j++ ) {
            console->printf("%.4f ", accel_affine(i,j));
        }
        console->printf("\n");
    }
}

// update the mag calibration matrix from the config structur
void imu_mgr_t::set_mag_calibration() {
    mag_affine = Eigen::Matrix4f::Identity();
    for ( int i = 0; i < 4; i++ ) {
        for ( int j = 0; j < 4; j++ ) {
            mag_affine(i,j) = imu_calib_node.getDouble("mag_affine", i*4+j);
        }
    }

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
void imu_mgr_t::init() {
    printf("imu_mgr.init()\n\n");
    hal.scheduler->delay(500);
    imu_node = PropertyNode("/sensors/imu");
    imu_calib_node = PropertyNode("/config/imu/calibration");
    hal.scheduler->delay(100);
    imu_hal.init();
}

// query the imu and update the structures
void imu_mgr_t::update() {
    // static uint8_t accel_count = ins.get_accel_count();
    // static uint8_t gyro_count = ins.get_gyro_count();

    string request = imu_node.getString("request");
    if ( request == "calibrate-accels" ) {
        imu_node.setString("request", "received: calibrate-accels");
        calib_accels.init();
    }
    
    imu_hal.update();
    imu_millis = imu_hal.raw_millis;
    
    accels_raw << imu_hal.accel.x, imu_hal.accel.y, imu_hal.accel.z, 1.0;
    gyros_raw << imu_hal.gyro.x, imu_hal.gyro.y, imu_hal.gyro.z;
    temp_C = imu_hal.temp_C;

    Eigen::Vector3f mags_precal;
    mags_precal << imu_hal.mag.x, imu_hal.mag.y, imu_hal.mag.z;
    mags_raw.head(3) = strapdown * mags_precal;
    mags_raw(3) = 1.0;
    
    accels_cal = accel_affine * accels_raw;
    gyros_cal = strapdown * gyros_raw;

    //accels_cal(0) = ax_cal.calibrate(accels_nocal(0), temp_C);
    //accels_cal(1) = ay_cal.calibrate(accels_nocal(1), temp_C);
    //accels_cal(2) = az_cal.calibrate(accels_nocal(2), temp_C);
        
    mags_cal = mag_affine * mags_raw;
    
    if ( gyros_calibrated < 2 ) {
        calibrate_gyros();
    } else {
        gyros_cal -= gyro_startup_bias;
    }

    // publish
    imu_node.setUInt("millis", imu_millis);
    imu_node.setDouble("timestamp", imu_millis / 1000.0);
    imu_node.setDouble("ax_raw", accels_raw(0));
    imu_node.setDouble("ay_raw", accels_raw(1));
    imu_node.setDouble("az_raw", accels_raw(2));
    imu_node.setDouble("hx_raw", mags_raw(0));
    imu_node.setDouble("hy_raw", mags_raw(1));
    imu_node.setDouble("hz_raw", mags_raw(2));
    imu_node.setDouble("ax_mps2", accels_cal(0));
    imu_node.setDouble("ay_mps2", accels_cal(1));
    imu_node.setDouble("az_mps2", accels_cal(2));
    imu_node.setDouble("p_rps", gyros_cal(0));
    imu_node.setDouble("q_rps", gyros_cal(1));
    imu_node.setDouble("r_rps", gyros_cal(2));
    imu_node.setDouble("hx", mags_cal(0));
    imu_node.setDouble("hy", mags_cal(1));
    imu_node.setDouble("hz", mags_cal(2));
    imu_node.setDouble("temp_C", temp_C);

    calib_accels.update();      // run if requested
}

// stay alive for up to 15 seconds looking for agreement between a 1
// second low pass filter and a 0.1 second low pass filter.  If these
// agree (close enough) for 4 consecutive seconds, then we calibrate
// with the 1 sec low pass filter value.  If time expires, the
// calibration fails and we run with raw gyro values.
void imu_mgr_t::calibrate_gyros() {
    if ( gyros_calibrated == 0 ) {
        console->printf("Initialize gyro calibration: ");
        slow = gyros_cal;
        fast = gyros_cal;
        total_timer = AP_HAL::millis();
        good_timer = AP_HAL::millis();
        output_timer = AP_HAL::millis();
        gyros_calibrated = 1;
    }

    fast = 0.95 * fast + 0.05 * gyros_cal;
    slow = 0.995 * fast + 0.005 * gyros_cal;
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
