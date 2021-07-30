// IMU wrapper class

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "setup_board.h"

#include "eigen3/Eigen/Core"

#include "props2.h"

#include "calibration/calib_accels.h"
#include "imu_hal.h"
#include "cal_temp.h"

class imu_mgr_t {
    
private:

    imu_hal_t imu_hal;
    
    Eigen::Matrix3f strapdown = Eigen::Matrix3f::Identity();
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

    calib_accels_t calib_accels;
    
    PropertyNode imu_node;
    PropertyNode imu_calib_node;

public:
    
    // 0 = uncalibrated, 1 = calibration in progress, 2 = calibration finished
    int gyros_calibrated = 0;
    unsigned long imu_millis = 0;
    // raw/uncorrected sensor values
    Eigen::Vector4f accels_raw =  Eigen::Vector4f::Zero();
    Eigen::Vector3f gyros_raw =  Eigen::Vector3f::Zero();
    Eigen::Vector4f mags_raw =  Eigen::Vector4f::Zero();
    // rotation corrected sensor values
    //Vector3f accels_nocal = Vector3f::Zero();
    //Vector3f gyros_nocal = Vector3f::Zero();
    //Vector3f mags_nocal = Vector3f::Zero();
    Eigen::Vector4f accels_cal = Eigen::Vector4f::Zero();
    Eigen::Vector3f gyros_cal = Eigen::Vector3f::Zero();
    Eigen::Vector4f mags_cal = Eigen::Vector4f::Zero();
    float temp_C = 0.0;
    
    void defaults();
    void set_strapdown_calibration();
    void set_accel_calibration();
    void set_mag_calibration();
    void setup();
    void update();
};

extern imu_mgr_t imu_mgr;
