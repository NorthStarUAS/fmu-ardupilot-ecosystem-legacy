#include <math.h>

#include "config.h"
#include "gps_mgr.h"
#include "imu_mgr.h"

#include "nav.h"

static const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void nav_t::setup() {
    config.ekf_cfg.select = message::enum_nav::nav15; // force (fixme)
    #if defined(AURA_ONBOARD_EKF)
    console->printf("EKF: available  Current setting: ");
    if ( config.ekf_cfg.select == message::enum_nav::none ) {
        console->printf("none\n");
    } else if ( config.ekf_cfg.select == message::enum_nav::nav15 ) {
        console->printf("15 state ins/gns\n");
    } else if ( config.ekf_cfg.select == message::enum_nav::nav15_mag ) {
        console->printf("15 state ins/gns/mag\n");
    } else {
        console->printf("unknown setting/disabled\n");
    }
    #else
    console->printf("EKF: not available for Teensy 3.2\n");
    #endif
}

void nav_t::update() {
    #if defined(AURA_ONBOARD_EKF)
    IMUdata imu1;
    imu1.time = imu_mgr.imu_millis / 1000.0;
    imu1.p = imu_mgr.get_p_cal();
    imu1.q = imu_mgr.get_q_cal();
    imu1.r = imu_mgr.get_r_cal();
    imu1.ax = imu_mgr.get_ax_cal();
    imu1.ay = imu_mgr.get_ay_cal();
    imu1.az = imu_mgr.get_az_cal();
    imu1.hx = imu_mgr.get_hx_cal();
    imu1.hy = imu_mgr.get_hy_cal();
    imu1.hz = imu_mgr.get_hz_cal();
    
    GPSdata gps1;
    gps1.time = imu_mgr.imu_millis / 1000.0;
    gps1.unix_sec = gps1.time;
    const Location &loc = gps_mgr.gps.location();
    gps1.lat = loc.lat / 10000000.0;
    gps1.lon = loc.lng / 10000000.0;
    gps1.alt = loc.alt / 100.0;
    const Vector3f vel = gps_mgr.gps.velocity();
    gps1.vn = vel.x;
    gps1.ve = vel.y;
    gps1.vd = vel.z;
    gps1.unix_sec = gps_mgr.unix_sec;
    
    if ( !ekf_inited and gps_mgr.settle() ) {
        if ( config.ekf_cfg.select == message::enum_nav::nav15 ) {
            ekf.init(imu1, gps1);
        } else if ( config.ekf_cfg.select == message::enum_nav::nav15_mag ) {
            ekf_mag.init(imu1, gps1);
        }
        ekf_inited = true;
        console->printf("EKF: initialized\n");
    } else if ( ekf_inited ) {
        if ( config.ekf_cfg.select == message::enum_nav::nav15 ) {
            ekf.time_update(imu1);
        } else if ( config.ekf_cfg.select == message::enum_nav::nav15_mag ) {
            ekf_mag.time_update(imu1);
        }
        if ( gps_mgr.gps_millis > gps_last_millis ) {
            gps_last_millis = gps_mgr.gps_millis;
            if ( config.ekf_cfg.select == message::enum_nav::nav15 ) {
                ekf.measurement_update(gps1);
            } else if ( config.ekf_cfg.select == message::enum_nav::nav15_mag ) {
                ekf_mag.measurement_update(imu1, gps1);
            }
            status = 2;         // ok
        }
        if ( config.ekf_cfg.select == message::enum_nav::nav15 ) {
            data = ekf.get_nav();
        } else if ( config.ekf_cfg.select == message::enum_nav::nav15_mag ) {
            data = ekf_mag.get_nav();
        }

        // sanity checks in case degenerate input leads to the filter
        // blowing up.  look for nans (or even negative #'s) in the
        // covariance matrix.
        if ( std::isnan(data.Pp0) or std::isnan(data.Pv0) or std::isnan(data.Pa0)
             or (data.Pp0 < -0.1) or (data.Pv0 < -0.1) or (data.Pa0 < -0.1) ) {
            console->printf("filter blew up...\n");
            status = 0;
            reinit();
        }
        if ( AP_HAL::millis() - gps_last_millis >= 2000 ) {
            // last gps message > 2 seconds ago
            status = 1;         // no gps
        }
    } else {
        status = 0;             // not initialized
    }
    #endif // AURA_ONBOARD_EKF
}

void nav_t::reinit() {
    ekf_inited = false;
}

// global shared instance
nav_t nav;
