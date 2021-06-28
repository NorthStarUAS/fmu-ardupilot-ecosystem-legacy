#pragma once

#include <string>
using std::string;

#include "setup_board.h"

#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "props2.h"

#include "util/lowpass.h"

class calib_accels_t {

private:

    uint32_t last_millis = 0;
    int state = -1;
    bool armed = false;
    LowPassFilter ax_slow = LowPassFilter(1.0);
    LowPassFilter ax_fast = LowPassFilter(0.2);
    LowPassFilter ay_slow = LowPassFilter(1.0);
    LowPassFilter ay_fast = LowPassFilter(0.2);
    LowPassFilter az_slow = LowPassFilter(1.0);
    LowPassFilter az_fast = LowPassFilter(0.2);

    Eigen::MatrixXf Meas;
    bool checked[6] = {false};
    
    PropertyNode imu_node;
    
    int raw_up_axis( float ax, float ay, float az );
    inline bool new_axis(int axis) {
        if ( axis >= 0 and !checked[axis] ) {
            return true;
        } else {
            return false;
        }
    }
    void compute_affine();
    
public:

    void setup();
    void update();
    
};
