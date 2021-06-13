#pragma once

#include "setup_board.h"

#include <math.h>
#include "eigen3/Eigen/Core"

class mixer_t {
private:
    Eigen::Matrix<float, MAX_RCOUT_CHANNELS, MAX_RCOUT_CHANNELS> M;
    Eigen::Matrix<float, MAX_RCOUT_CHANNELS, 1> inputs;
    
    void sas_update();
    void mixing_update();

public:
    Eigen::Matrix<float, MAX_RCOUT_CHANNELS, 1> outputs;

    void print_mixer_matrix();
    void setup();
    void sas_defaults();
    //void mixing_defaults();
    void update_matrix(message::config_mixer_t *mix_config );
    void update();
};
