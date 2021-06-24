#include <math.h>

#include "nav_mgr.h"

void nav_mgr_t::setup() {
    config_ekf_node = PropertyNode("/config/ekf");
    gps_node = PropertyNode("/sensors/gps");
    imu_node = PropertyNode("/sensors/imu");
    nav_node = PropertyNode("/filters/nav");
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
    configure();
}

void nav_mgr_t::configure() {
    string selected = config_ekf_node.getString("select");
    NAVconfig config;
    if ( selected == "nav15" ) {
        config = ekf.get_config();
    } else if ( selected == "nav15_mag" ) {
        config = ekf_mag.get_config();
    }
    if ( config_ekf_node.hasChild("sig_w_accel") ) {
        config.sig_w_ax = config_ekf_node.getDouble("sig_w_accel");
        config.sig_w_ay = config.sig_w_ax;
        config.sig_w_az = config.sig_w_ax;
    }
    if ( config_ekf_node.hasChild("sig_w_gyro") ) {
        config.sig_w_gx = config_ekf_node.getDouble("sig_w_gyro");
        config.sig_w_gy = config.sig_w_gx;
        config.sig_w_gz = config.sig_w_gx;
    }
    if ( config_ekf_node.hasChild("sig_a_d") ) {
        config.sig_a_d = config_ekf_node.getDouble("sig_a_d");
    }
    if ( config_ekf_node.hasChild("tau_a") ) {
        config.tau_a = config_ekf_node.getDouble("tau_a");
    }
    if ( config_ekf_node.hasChild("sig_g_d") ) {
        config.sig_g_d = config_ekf_node.getDouble("sig_g_d");
    }
    if ( config_ekf_node.hasChild("tau_g") ) {
        config.tau_g = config_ekf_node.getDouble("tau_g");
    }
    if ( config_ekf_node.hasChild("sig_gps_p_ne") ) {
        config.sig_gps_p_ne = config_ekf_node.getDouble("sig_gps_p_ne");
    }
    if ( config_ekf_node.hasChild("sig_gps_p_d") ) {
        config.sig_gps_p_d = config_ekf_node.getDouble("sig_gps_p_d");
    }
    if ( config_ekf_node.hasChild("sig_gps_v_ne") ) {
        config.sig_gps_v_ne = config_ekf_node.getDouble("sig_gps_v_ne");
    }
    if ( config_ekf_node.hasChild("sig_gps_v_d") ) {
        config.sig_gps_v_d = config_ekf_node.getDouble("sig_gps_v_d");
    }
    if ( config_ekf_node.hasChild("sig_mag") ) {
        config.sig_mag = config_ekf_node.getDouble("sig_mag");
    }
    if ( selected == "nav15" ) {
        ekf.set_config(config);
    } else if ( selected == "nav15_mag" ) {
        ekf_mag.set_config(config);
    }
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
    gps1.time = gps_node.getFloat("timestamp");
    gps1.unix_sec = gps_node.getFloat("unix_sec");
    gps1.lat = gps_node.getDouble("latitude_deg");
    gps1.lon = gps_node.getDouble("longitude_deg");
    gps1.alt = gps_node.getFloat("altitude_m");
    gps1.vn = gps_node.getFloat("vn_mps");
    gps1.ve = gps_node.getFloat("ve_mps");
    gps1.vd = gps_node.getFloat("vd_mps");

    string selected = config_ekf_node.getString("select");
    if ( !ekf_inited and gps_node.getBool("settle") ) {
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
        if ( gps_node.getUInt("millis") > gps_last_millis ) {
            gps_last_millis = gps_node.getUInt("millis");
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
        if ( std::isnan(data.Pp0) or std::isnan(data.Pv0)
             or std::isnan(data.Pa0) or (data.Pp0 < -0.1)
             or (data.Pv0 < -0.1) or (data.Pa0 < -0.1) ) {
            console->printf("filter blew up...\n");
            status = 0;
            reinit();
        }
        if ( AP_HAL::millis() - gps_last_millis >= 2000 ) {
            // last gps message > 2 seconds ago
            status = 1;         // no gps
        }
        
        // publish
        nav_node.setDouble("latitude_rad", data.lat);
        nav_node.setDouble("longitude_rad", data.lon);
        nav_node.setFloat("altitude_m", data.alt);
        nav_node.setFloat("vn_mps", data.vn);
        nav_node.setFloat("ve_mps", data.ve);
        nav_node.setFloat("vd_mps", data.vd);
        nav_node.setFloat("phi_rad", data.phi);
        nav_node.setFloat("the_rad", data.the);
        nav_node.setFloat("psi_rad", data.psi);
        nav_node.setFloat("p_bias", data.gbx);
        nav_node.setFloat("q_bias", data.gby);
        nav_node.setFloat("r_bias", data.gbz);
        nav_node.setFloat("ax_bias", data.abx);
        nav_node.setFloat("ay_bias", data.aby);
        nav_node.setFloat("az_bias", data.abz);
        nav_node.setFloat("Pp0", data.Pp0);
        nav_node.setFloat("Pp1", data.Pp1);
        nav_node.setFloat("Pp2", data.Pp2);
        nav_node.setFloat("Pv0", data.Pv0);
        nav_node.setFloat("Pv1", data.Pv1);
        nav_node.setFloat("Pv2", data.Pv2);
        nav_node.setFloat("Pa0", data.Pa0);
        nav_node.setFloat("Pa1", data.Pa1);
        nav_node.setFloat("Pa2", data.Pa2);
    } else {
        status = 0;             // not initialized
    }
    nav_node.setInt("status", status);
#endif // AURA_ONBOARD_EKF
}

void nav_mgr_t::reinit() {
    ekf_inited = false;
}

// global shared instance
nav_mgr_t nav_mgr;
