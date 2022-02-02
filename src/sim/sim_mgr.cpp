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
    
    const char *file_path = "fdm.json";
    if ( !sim_node.load(file_path) ) {
        console->printf("FDM file loading failed: %s\n", file_path);
    }

    // populate the A state transition matrix
    num_states = sim_node.getLen("parameters");
    A.resize(num_states, num_states);
    for ( int i = 0; i < num_states; i++ ) {
        for ( int j = 0; j < num_states; j++ ) {
            A(i,j) = sim_node.getDouble("A", i*num_states+j);
        }
    }
    
    printf("A state transition matrix: %d x %d\n", num_states, num_states);
    // for ( int i = 0; i < num_states; i++ ) {
    //     printf("  ");
    //     for ( int j = 0; j < num_states; j++ ) {
    //         printf("%.3f ", A(i,j));
    //     }
    //     printf("\n");
    // }

    state.resize(num_states, 1);
    
    hal.scheduler->delay(200);
}

void sim_mgr_t::reset() {
    float initial_airspeed_mps = 10.0;
    set_airdata( initial_airspeed_mps );
    set_throttle( 0.5 );
    set_flight_surfaces( 0.0, -0.1, 0.0, 0.0 );
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
    for ( int i = 0; i < num_states; i++ ) {
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
        string field_type = param.getString("type");
        if ( field_type == "dependent" ) {
            float min = param.getDouble("min");
            float max = param.getDouble("max");
            float std = param.getDouble("std");
            int n = 1;
            if ( val < min - n*std ) {
                val = min - n*std;
                printf("  %s clipped to: %.3f", field.c_str(), val);
            }
            if (val > max + n*std ) {
                val = max + n*std;
                printf("  %s clipped to: %.3f", field.c_str(), val);
            }
        }
        state[i] = val;
    }
}

void sim_mgr_t::from_state_vector( Eigen::MatrixXf next_state ) {
    for ( int i = 0; i < num_states; i++ ) {
        string param_name = "parameters/" + std::to_string(i);
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
            accel_body[0] = next_state[i];
        } else if ( field == "bay" ) {
            accel_body[1] = next_state[i];
        } else if ( field == "baz" ) {
            accel_body[2] = next_state[i];
        } else if ( field == "airspeed" ) {
            set_airdata( next_state[i] );
        } else if ( field == "bvx" ) {
            vel_body[0] = next_state[i];
        } else if ( field == "bvy" ) {
            vel_body[1] = next_state[i];
        } else if ( field == "bvz" ) {
            vel_body[2] = next_state[i];
        } else if ( field == "p" ) {
            p = next_state[i];
        } else if ( field == "q" ) {
            q = next_state[i];
        } else if ( field == "r" ) {
            r = next_state[i];;
        } else {
            printf("Unknown field requested: %s gack!", field.c_str());
        }
    }
}

void sim_mgr_t::update() {
    to_state_vector();
    Eigen::MatrixXf next = A * state;
    //add_noise(next)
    from_state_vector(next);
        
    // update body frame velocity from accel estimates (* dt)
            self.bvx += result["bax"] * self.dt
            self.bvy += result["bay"] * self.dt
            self.bvz += result["baz"] * self.dt
            // force bvx to be positive and non-zero (no tail slides here)
            if self.bvx < 0.1:
                self.bvx = 0.1
            // try to clamp alpha/beta from getting crazy
            if abs(self.bvy / self.bvx) > 0.1:
                self.bvy = np.sign(self.bvy) * abs(self.bvx) * 0.1
            if abs(self.bvz / self.bvx) > 0.1:
                self.bvz = np.sign(self.bvz) * abs(self.bvx) * 0.1
            // scale to airspeed
            v = np.array( [self.bvx, self.bvy, self.bvz] )
            v *= (self.airspeed_mps / np.linalg.norm(v))
            self.bvx = v[0]
            self.bvy = v[1]
            self.bvz = v[2]
            self.state_mgr.set_body_velocity( v[0], v[1], v[2] )
            self.alpha = atan2( self.bvz, self.bvx )
            self.beta = atan2( -self.bvy, self.bvx )
        
        // update attitude
        rot_body = quaternion.eul2quat(self.p * self.dt,
                                       self.q * self.dt,
                                       self.r * self.dt)
        self.ned2body = quaternion.multiply(self.ned2body, rot_body)
        self.phi_rad, self.the_rad, self.psi_rad = quaternion.quat2eul(self.ned2body)
        self.state_mgr.set_orientation(self.phi_rad, self.the_rad, self.psi_rad)

        self.state_mgr.compute_body_frame_values(compute_body_vel=False)
        
        // velocity in ned frame
        self.vel_ned = quaternion.backTransform( self.ned2body,
                                                 np.array([self.bvx, self.bvy, self.bvz]) )
        self.state_mgr.set_ned_velocity( self.vel_ned[0],
                                         self.vel_ned[1],
                                         self.vel_ned[2],
                                         0.0, 0.0, 0.0 )

        // update position
        self.pos_ned += self.vel_ned * self.dt

        // store data point
        self.data.append(
            [ self.time, self.airspeed_mps,
              self.state_mgr.throttle,
              self.state_mgr.aileron,
              self.state_mgr.elevator,
              self.state_mgr.rudder,
              self.phi_rad, self.the_rad, self.psi_rad,
              self.state_mgr.alpha, self.state_mgr.beta,
              self.p, self.q, self.r] )
        self.data[-1].extend( self.pos_ned.tolist() )
        self.data[-1].extend( self.vel_ned.tolist() )

        // update time
        self.time += self.dt
                    }
