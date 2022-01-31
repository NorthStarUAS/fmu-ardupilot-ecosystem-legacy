// Onboard simulation manager class

#pragma once

#include <AP_HAL/AP_HAL.h>

#include "setup_board.h"

#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "props2.h"

class sim_mgr_t {
    
public:
    void init();
    void update();
    
private:
    PropertyNode sim_node;
    Eigen::MatrixXf A;          // state transition matrix
};
