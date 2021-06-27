#pragma once

#include "setup_board.h"

#include <math.h>
#include "eigen3/Eigen/Core"

#include "props2.h"

class mixer_t {

private:
    Eigen::MatrixXf M;
    Eigen::VectorXf inputs, outputs;
    
    void sas_update();
    void mixing_update();

    PropertyNode effector_node;
    PropertyNode imu_node;
    PropertyNode pilot_node;
    PropertyNode stab_roll_node;
    PropertyNode stab_pitch_node;
    PropertyNode stab_yaw_node;
    PropertyNode stab_tune_node;
    
public:

    void print_mixer_matrix();
    void setup();
    void sas_defaults();
    //void mixing_defaults();
    void update_matrix();
    void update();
};
