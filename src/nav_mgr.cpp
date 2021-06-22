#include <math.h>

#include "gps_mgr.h"

#include "nav_mgr.h"

void nav_mgr_t::setup() {
    config_ekf_node = PropertyNode("/config/ekf");
    imu_node = PropertyNode("/sensors/imu");
    string selected = config_ekf_node.getString("select");
    // fix me ...
    if ( selected == ""  ) {
        selected = "none";
        config_ekf_node.setString("select", selected);
    } else {
        selected = "nav15";         // force (fixme)
    }
#if defined(AURA_ONBOARD_EKF)
    console->printf("EKF: selected: %s\n", selected.c_str());
#else
    console->printf("EKF: not available for Teensy 3.2\n");
#endif
}

void nav_mgr_t::update() {
#if defined(AURA_ONBOARD_EKF)
    IMUdata imu1;
    imu1.time = imu_node.getFloat("timestamp");
    imu1.p = imu_node.getFloat("p_rps");
    imu1.q = imu_node.getFloat("q_rps");
    imu1.r = imu_node.getFloat("r_rps");
    imu1.ax = imu_node.getFloat("ax_mps2");
    imu1.ay = imu_node.getFloat("ay_mps2");
    imu1.az = imu_node.getFloat("az_mps2");
    imu1.hx = imu_node.getFloat("hx");
    imu1.hy = imu_node.getFloat("hy");
    imu1.hz = imu_node.getFloat("hz");
    
    GPSdata gps1;
    gps1.time = imu_node.getFloat("timestamp"); // fixme
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

    string selected = config_ekf_node.getString("selected");
    if ( !ekf_inited and gps_mgr.settle() ) {
        if ( selected == "nav15" ) {
            ekf.init(imu1, gps1);
        } else if ( selected == "nav15_mag" ) {
            ekf_mag.init(imu1, gps1);
        }
        ekf_inited = true;
        console->printf("EKF: initialized\n");
    } else if ( ekf_inited ) {
        if ( selected == "nav15" ) {
            ekf.time_update(imu1);
        } else if ( selected == "nav15_mag" ) {
            ekf_mag.time_update(imu1);
        }
        if ( gps_mgr.gps_millis > gps_last_millis ) {
            gps_last_millis = gps_mgr.gps_millis;
            if ( selected == "nav15" ) {
                ekf.measurement_update(gps1);
            } else if ( selected == "nav15_mag" ) {
                ekf_mag.measurement_update(imu1, gps1);
            }
            status = 2;         // ok
        }
        if ( selected == "nav15" ) {
            data = ekf.get_nav();
        } else if ( selected == "nav15_mag" ) {
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

void nav_mgr_t::reinit() {
    ekf_inited = false;
}

// global shared instance
nav_mgr_t nav_mgr;
