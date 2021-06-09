#pragma once

#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/LU"

#include <math.h>

#include "pwm.h"
#include "sbus.h"

class mixer_t {
private:
    Eigen::Matrix<float, MAX_PWM_CHANNELS, MAX_PWM_CHANNELS> M;
    Eigen::Matrix<float, MAX_PWM_CHANNELS, 1> inputs;
    
    void sas_update();
    void mixing_update();

public:
    Eigen::Matrix<float, MAX_PWM_CHANNELS, 1> outputs;

    void print_mixer_matrix();
    void setup();
    void sas_defaults();
    void update_matrix(message::config_mixer_t *mix_config );
    void update( float control_norm[SBUS_CHANNELS] );
};

extern mixer_t mixer;
