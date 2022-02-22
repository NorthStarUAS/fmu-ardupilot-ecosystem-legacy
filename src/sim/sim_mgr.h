// Onboard simulation manager class

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "nav/nav_constants.h"

#include "setup_board.h"

#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "props2.h"

class sim_mgr_t {
    
public:
    void init();
    void reset();
    void run_loop();
    void update();
    
private:
    void set_airdata( float vel_mps ) {
        airspeed_mps = vel_mps;
        if ( airspeed_mps < 0.0 ) { airspeed_mps = 0.0; }
        qbar = 0.5 * airspeed_mps * airspeed_mps;
    }
    void set_throttle( float val ) {
        throttle = val;
        if ( throttle < 0.0 ) { throttle = 0.0; }
        if ( throttle > 1.0 ) { throttle = 1.0; }
        thrust = sqrt(throttle) * 0.75 * fabs(GRAVITY_NOM);
    }
    void set_flight_surfaces( float ail, float ele, float rud, float flp ) {
        aileron = ail;
        elevator = ele;
        rudder = rud;
        flaps = flp;
        if ( aileron < -1.0 ) { aileron = -1.0; }
        if ( aileron > 1.0 ) { aileron = 1.0; }
        if ( elevator < -1.0 ) { elevator = -1.0; }
        if ( elevator > 1.0 ) { elevator = 1.0; }
        if ( rudder < -1.0 ) { rudder = -1.0; }
        if ( rudder > 1.0 ) { rudder = 1.0; }
        if ( flaps < 0.0 ) { flaps = 0.0; }
        if ( flaps > 1.0 ) { flaps = 1.0; }
    }
    void to_state_vector();
    void from_state_vector( Eigen::MatrixXf next_state );
    
    PropertyNode sim_node;
    int rows = 0;
    int cols = 0;
    Eigen::MatrixXf A;          // state transition matrix
    Eigen::MatrixXf state;      // state vector
    uint32_t dt_millis = 10;
    uint32_t sim_millis = 0;
    
    float airspeed_mps = 0.0;
    float qbar = 0.0;
    float lift = 0.0;
    float drag = 0.0;
    float Cl = 0.0;
    float Cd = 0.0;
    
    float throttle = 0.0;
    float thrust = 0.0;
    float aileron = 0.0;
    float elevator = 0.0;
    float rudder = 0.0;
    float flaps = 0.0;

    Eigen::Vector3f pos_ned;
    Eigen::Vector3f vel_ned;
    Eigen::Vector3f vel_body;
    Eigen::Vector3f vel_flow;
    
    float phi_rad = 0.0;
    float the_rad = 0.0;
    float psi_rad = 0.0;
    Eigen::Quaternionf ned2body;

    Eigen::Vector3f accel_body;
    Eigen::Vector3f g_body;
    float p = 0.0;
    float q = 0.0;
    float r = 0.0;
    float ax = 0.0;
    float ay = 0.0;
    float az = 0.0;

    float alpha = 0.0;
    float beta = 0.0;
};
