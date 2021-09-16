#include <math.h>

#include "wind.h"

// initialize wind estimator variables
void wind_est_t::init() {
    airdata_node = PropertyNode( "/sensors/airdata/0" );
    filter_node = PropertyNode( "/filters/nav" );
    orient_node = PropertyNode( "/orientation" );
    vel_node = PropertyNode( "/velocity" );
    wind_node = PropertyNode( "/sensors/airdata/0" );
    
    wind_node.setDouble( "pitot_scale_factor", 1.0 );

    we_filt.set_time_factor(60.0);
    wn_filt.set_time_factor(60.0);
    pitot_scale_filt.set_time_factor(120.0);
    pitot_scale_filt.init(1.0);
}


// onboard wind estimate (requires airspeed, true heading, and ground
// velocity vector)
void wind_est_t::update( double dt ) {
    const double d2r = M_PI / 180.0; // degrees to radians
    const double r2d = 180.0 / M_PI; // radians to degrees
    const double mps2kt = 1.9438444924406046432;
    const double kt2mps = 0.5144444444444444444;

    double airspeed_kt = airdata_node.getDouble("airspeed_kt");
    if ( ! airdata_node.getBool("is_airborne") ) {
	// The wind estimator only works for fixed wing flight.
	return;
    }

    double psi = M_PI_2 - orient_node.getDouble("heading_deg") * d2r;
    double pitot_scale = pitot_scale_filt.get_value();
    double ue = cos(psi) * (airspeed_kt * pitot_scale * kt2mps);
    double un = sin(psi) * (airspeed_kt * pitot_scale * kt2mps);
    double we = ue - filter_node.getDouble("ve_mps");
    double wn = un - filter_node.getDouble("vn_mps");

    we_filt.update(we, dt);
    wn_filt.update(wn, dt);

    double we_filt_val = we_filt.get_value();
    double wn_filt_val = wn_filt.get_value();
    
    double wind_deg = 90 - atan2( wn_filt_val, we_filt_val ) * r2d;
    if ( wind_deg < 0 ) {
        wind_deg += 360.0;
    }
    double wind_speed_kt = sqrt( we_filt_val*we_filt_val
				 + wn_filt_val*wn_filt_val ) * mps2kt;

    wind_node.setDouble( "wind_speed_kt", wind_speed_kt );
    wind_node.setDouble( "wind_dir_deg", wind_deg );
    wind_node.setDouble( "wind_east_mps", we_filt_val );
    wind_node.setDouble( "wind_north_mps", wn_filt_val );

    // estimate pitot tube bias
    double true_e = we_filt_val + filter_node.getDouble("ve_mps");
    double true_n = wn_filt_val + filter_node.getDouble("vn_mps");

    double true_deg = 90 - atan2( true_n, true_e ) * r2d;
    if ( true_deg < 0 ) {
        true_deg += 360.0;
    }
    double true_speed_kt = sqrt( true_e*true_e + true_n*true_n ) * mps2kt;

    wind_node.setDouble( "true_airspeed_kt", true_speed_kt );
    wind_node.setDouble( "true_heading_deg", true_deg );
    wind_node.setDouble( "true_airspeed_east_mps", true_e );
    wind_node.setDouble( "true_airspeed_north_mps", true_n );

    double ps = 1.0;
    if ( airspeed_kt > 1.0 ) {
	ps = true_speed_kt / airspeed_kt;
	// don't let the scale factor exceed some reasonable limits
	if ( ps < 0.75 ) { ps = 0.75;	}
	if ( ps > 1.25 ) { ps = 1.25; }
    }

    pitot_scale_filt.update(ps, dt);
    wind_node.setDouble( "pitot_scale_factor", pitot_scale_filt.get_value() );

    // if ( display_on ) {
    //   printf("true: %.2f kt  %.1f deg (scale = %.4f)\n", true_speed_kt, true_deg, pitot_scale_filt);
    // }

    // now estimate ground speed/track based on airdata and wind estimate
    double ve_est = ue - we_filt_val;
    double vn_est = un - wn_filt_val;
    double groundtrack_est_deg = 90 - atan2( vn_est, ve_est ) * r2d;
    if ( groundtrack_est_deg < 0 ) {
        groundtrack_est_deg += 360.0;
    }
    double groundspeed_est_ms = sqrt( ve_est*ve_est + vn_est*vn_est );
    // double groundspeed_est_kt = groundspeed_est_ms * SG_MPS_TO_KT;
    vel_node.setDouble( "groundspeed_est_ms", groundspeed_est_ms );
    orient_node.setDouble( "groundtrack_est_deg", groundtrack_est_deg );
}
