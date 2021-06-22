// IMU wrapper class

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "setup_board.h"

#include "eigen3/Eigen/Core"

#include "props2.h"

#include "imu_hal.h"
#include "cal_temp.h"

class imu_mgr_t {
    
private:

    imu_hal_t imu_hal;
    
    Eigen::Matrix4f strapdown = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f accel_affine = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f mag_affine = Eigen::Matrix4f::Identity();
    // gyro zero'ing stuff
    const float cutoff = 0.005;
    uint32_t total_timer = 0;
    uint32_t good_timer = 0;
    uint32_t output_timer = 0;
    Eigen::Vector3f slow = Eigen::Vector3f::Zero();
    Eigen::Vector3f fast = Eigen::Vector3f::Zero();
    Eigen::Vector3f gyro_startup_bias = Eigen::Vector3f::Zero();
    void calibrate_gyros();
    CalTemp ax_cal;
    CalTemp ay_cal;
    CalTemp az_cal;

    PropertyNode imu_node;
    PropertyNode imu_calib_node;

public:
    
    // 0 = uncalibrated, 1 = calibration in progress, 2 = calibration finished
    int gyros_calibrated = 0;
    unsigned long imu_millis = 0;
    // raw/uncorrected sensor values
    Eigen::Vector4f accels_raw =  Eigen::Vector4f::Zero();
    Eigen::Vector4f gyros_raw =  Eigen::Vector4f::Zero();
    Eigen::Vector4f mags_raw =  Eigen::Vector4f::Zero();
    // rotation corrected sensor values
    //Vector3f accels_nocal = Vector3f::Zero();
    //Vector3f gyros_nocal = Vector3f::Zero();
    //Vector3f mags_nocal = Vector3f::Zero();
    Eigen::Vector4f accels_cal = Eigen::Vector4f::Zero();
    Eigen::Vector4f gyros_cal = Eigen::Vector4f::Zero();
    Eigen::Vector4f mags_cal = Eigen::Vector4f::Zero();
    float tempC = 0.0;
    
    void defaults();
    void set_strapdown_calibration();
    // void set_accel_calibration();
    void set_mag_calibration();
    void setup();
    void update();
    // notational convenience/clarity
    inline float get_ax_raw() { return accels_raw(0); }
    inline float get_ay_raw() { return accels_raw(1); }
    inline float get_az_raw() { return accels_raw(2); }
    inline float get_p_raw() { return gyros_raw(0); }
    inline float get_q_raw() { return gyros_raw(1); }
    inline float get_r_raw() { return gyros_raw(2); }
    inline float get_hx_raw() { return mags_raw(0); }
    inline float get_hy_raw() { return mags_raw(1); }
    inline float get_hz_raw() { return mags_raw(2); }
    inline float get_ax_cal() { return accels_cal(0); }
    inline float get_ay_cal() { return accels_cal(1); }
    inline float get_az_cal() { return accels_cal(2); }
    inline float get_p_cal() { return gyros_cal(0); }
    inline float get_q_cal() { return gyros_cal(1); }
    inline float get_r_cal() { return gyros_cal(2); }
    inline float get_hx_cal() { return mags_cal(0); }
    inline float get_hy_cal() { return mags_cal(1); }
    inline float get_hz_cal() { return mags_cal(2); }
    inline float get_tempC() { return tempC; }
};

extern imu_mgr_t imu_mgr;
