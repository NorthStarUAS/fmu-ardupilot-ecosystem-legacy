#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>

#include "setup_board.h"

#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include <string>
using std::string;

#include "nav/nav_functions.h"

#include "sim_mgr.h"

void sim_mgr_t::init() {
    sim_node = PropertyNode("/sim");
    pilot_node = PropertyNode("/pilot");
    
    const char *file_path = "fdm.json";
    if ( !sim_node.load(file_path) ) {
        console->printf("FDM file loading failed: %s\n", file_path);
    }

    // populate the A state transition matrix
    dt = sim_node.getDouble("dt");
    dt_millis = (uint32_t)(dt * 1000 + 0.5); // round to nearest us
    rows = sim_node.getInt("rows");
    cols = sim_node.getInt("cols");
    A.resize(rows, cols);
    for ( int i = 0; i < rows; i++ ) {
        for ( int j = 0; j < cols; j++ ) {
            A(i,j) = sim_node.getDouble("A", i*cols+j);
        }
    }
    
    printf("A state transition matrix: %d x %d\n", rows, cols);
    // for ( int i = 0; i < num_states; i++ ) {
    //     printf("  ");
    //     for ( int j = 0; j < num_states; j++ ) {
    //         printf("%.3f ", A(i,j));
    //     }
    //     printf("\n");
    // }

    state.resize(cols, 1);
    
    hal.scheduler->delay(200);
}

void sim_mgr_t::reset() {
    float initial_airspeed_mps = 15.0;
    set_airdata( initial_airspeed_mps );
    set_throttle( 0.5 );
    set_flight_surfaces( 0.0, -0.01, 0.0, 0.0 );
    pos_ned << 0.0, 0.0, 0.0;
    vel_ned << initial_airspeed_mps, 0.0, 0.0;
    vel_body << initial_airspeed_mps, 0.0, 0.0;
    phi_rad = 0.0;
    the_rad = 0.0;
    psi_rad = 0.0;
    ned2body = eul2quat( phi_rad, the_rad, psi_rad );
    p = 0.0;
    q = 0.0;
    r = 0.0;
    ax = 0.0;
    ay = 0.0;
    az = 0.0;
}

void sim_mgr_t::to_state_vector() {
    for ( int i = 0; i < cols; i++ ) {
        string param_name = "parameters/" + std::to_string(i);
        PropertyNode param = sim_node.getChild(param_name.c_str());
        if ( param.isNull() ) {
            continue;
        }
        string field = param.getString("name");
        float val = 0;
        if ( field == "aileron" ) {
            val = aileron * qbar;
        } else if ( field == "abs(aileron)" ) {
            val = fabs(aileron) * qbar;
        } else if ( field == "elevator" ) {
            val = elevator * qbar;
        } else if ( field == "rudder" ) {
            val = rudder * qbar;
        } else if ( field == "abs(rudder)" ) {
            val = abs(rudder) * qbar;
        } else if ( field == "flaps" ) {
            val = flaps * qbar;
        } else if ( field == "drag" ) {
            val = drag;
        } else if ( field == "thrust" ) {
            val = thrust;
        } else if ( field == "bgx" ) {
            val = g_body[0];
        } else if ( field == "bgy" ) {
            val = g_body[1];
        } else if ( field == "bgz" ) {
            val = g_body[2];
        } else if ( field == "bax" ) {
            val = accel_body[0];
        } else if ( field == "bay" ) {
            val = accel_body[1];
        } else if ( field == "baz" ) {
            val = accel_body[2];
        } else if ( field == "airspeed" ) {
            val = airspeed_mps;
        } else if ( field == "bvx" ) {
            val = vel_body[0];
        } else if ( field == "bvy" ) {
            val = vel_body[1];
        } else if ( field == "bvz" ) {
            val = vel_body[2];
        } else if ( field == "p" ) {
            val = p;
        } else if ( field == "q" ) {
            val = q;
        } else if ( field == "r" ) {
            val = r;
        } else {
            printf("Unknown field requested: %s gack!", field.c_str());
        }
	//printf("%d: %.1f ", i, val);
	//hal.scheduler->delay(50);
        string field_type = param.getString("type");
        if ( field_type == "dependent" ) {
            float min = param.getDouble("min");
            float max = param.getDouble("max");
            float std = param.getDouble("std");
            int n = 1;
            if ( val < min - n*std ) {
                val = min - n*std;
                //printf("  %s clipped to: %.3f\n", field.c_str(), val);
            }
            if (val > max + n*std ) {
                val = max + n*std;
                //printf("  %s clipped to: %.3f\n", field.c_str(), val);
            }
        }
        state(i) = val;
    }
    //printf("\n");
}

void sim_mgr_t::from_state_vector( Eigen::MatrixXf next_state ) {
    for ( int i = 0; i < rows; i++ ) {
        string param_name = "parameters/" + std::to_string(i + (cols-rows));
        PropertyNode param = sim_node.getChild(param_name.c_str());
        if ( param.isNull() ) {
            continue;
        }
        string field = param.getString("name");
        string field_type = param.getString("type");
        if ( field_type != "dependent" ) {
            continue;
        }
        if ( field == "bax" ) {
            accel_body(0) = next_state(i);
        } else if ( field == "bay" ) {
            accel_body(1) = next_state(i);
        } else if ( field == "baz" ) {
            accel_body(2) = next_state(i);
        } else if ( field == "airspeed" ) {
            set_airdata( next_state(i) );
        } else if ( field == "bvx" ) {
            vel_body(0) = next_state(i);
        } else if ( field == "bvy" ) {
            vel_body(1) = next_state(i);
        } else if ( field == "bvz" ) {
            vel_body(2) = next_state(i);
        } else if ( field == "p" ) {
            p = next_state(i);
        } else if ( field == "q" ) {
            q = next_state(i);
        } else if ( field == "r" ) {
            r = next_state(i);
        } else {
            printf("Unknown field requested: %s gack!", field.c_str());
        }
	//printf("%d: %.1f ", i, next_state(i));
    }
    //printf("\n");
}

inline float sign(float x) {
    return (x > 0.0) - (x < 0.0);
}

void sim_mgr_t::run_loop() {
    to_state_vector();
    Eigen::MatrixXf next = A * state;
    //add_noise(next)
    from_state_vector(next);
        
    // update body frame velocity from accel estimates (* dt)
    vel_body(0) += accel_body[0] * dt;
    vel_body(1) += accel_body[1] * dt;
    vel_body(2) += accel_body[2] * dt;
    //printf("velb: %.1f %.1f %.1f\n", vel_body(0), vel_body(1), vel_body(2));
    //hal.scheduler->delay(50);
    
    // force bvx to be positive and non-zero (no tail slides here)
    if ( vel_body[0] < 0.1 ) { vel_body[0] = 0.1; }
    // try to clamp alpha/beta from getting crazy
    if ( fabs(vel_body[1] / vel_body[0]) > 0.1 ) {
        vel_body(1) = sign(vel_body[1]) * fabs(vel_body[0]) * 0.1;
    }
    if ( fabs(vel_body[2] / vel_body[0]) > 0.1 ) {
        vel_body(2) = sign(vel_body[2]) * fabs(vel_body[0]) * 0.1;
    }
    // scale to airspeed
    vel_body *= (airspeed_mps / vel_body.norm());
    alpha = atan2( vel_body[2], vel_body[0] );
    beta = atan2( -vel_body[1], vel_body[0] );
        
    // update attitude
    Eigen::Quaternionf rot_body = eul2quat(p * dt, q * dt, r * dt);
    ned2body = ned2body * rot_body;
    Eigen::Vector3f eul = quat2eul( ned2body );
    phi_rad = eul[0];
    the_rad = eul[1];
    psi_rad = eul[2];

    // ned velocity
    vel_ned = ned2body.inverse() * vel_body;
    
    // flow (wind) frame of reference
    Eigen::Quaternionf body2flow = eul2quat(0.0, -alpha, -beta);
    Eigen::Quaternionf ned2flow = ned2body * body2flow;
    vel_flow = ned2flow * vel_ned; // fixme, how is vel_ned set?

    // lift, drag, and weight vector estimates

    // rotate ned gravity vector into body frame
    Eigen::Vector3f g_ned;
    g_ned << 0.0, 0.0, GRAVITY_NOM;
    g_body = ned2body * g_ned;

    // rotate ned gravity vector into flow frame
    Eigen::Vector3f g_flow = ned2flow * g_ned;
    Eigen::Vector3f accel_flow = body2flow * accel_body;
        
    // is my math correct here? (for drag need grav & body_accel in
    // flight path frame of reference ... I think.
        
    drag = (thrust - g_flow[0]) - accel_flow[0];
    if ( qbar > 10 ) {
        Cd = drag / qbar;
    } else {
        Cd = 0;
    }

    // do I need to think through lift frame of reference here too?
        
    lift = -accel_flow[2] - g_flow[2];
    if ( qbar > 10 ) {
        Cl = lift / qbar;
    } else {
        Cl = 0;
    }
    // print(" lift:", self.a_flow[2], self.g_flow[2], self.lift)
    
    // update position
    pos_ned += vel_ned * dt;
}

void sim_mgr_t::update() {
    uint32_t millis = AP_HAL::millis();
    
    if ( millis - sim_millis > 100 ) {
        // catchup and run an iteration
        sim_millis = millis - dt_millis;
    }

    set_throttle( pilot_node.getDouble("throttle") );
    set_flight_surfaces( pilot_node.getDouble("aileron"),
                         pilot_node.getDouble("elevator"),
                         pilot_node.getDouble("rudder"),
                         pilot_node.getDouble("flaps") );
    printf("pilot: %.2f %.2f %.2f ", aileron, elevator, throttle);
    while ( sim_millis < millis ) {
        run_loop();
        sim_millis += dt_millis;
    }

    static const double r2d = 180.0 / M_PI;
    printf("roll: %.1f  pitch: %.1f ", phi_rad*r2d, the_rad*r2d);
    printf("p: %.1f  q: %.1f  r: %.1f\n", p*r2d, q*r2d, r*r2d);
}
