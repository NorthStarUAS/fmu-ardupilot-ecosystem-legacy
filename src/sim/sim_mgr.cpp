#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>

#include "setup_board.h"

#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

#include "sim_mgr.h"

void sim_mgr_t::init() {
    sim_node = PropertyNode("/sim");
    
    const char *file_path = "fdm.json";
    if ( !sim_node.load(file_path) ) {
        console->printf("FDM file loading failed: %s\n", file_path);
    }

    // populate the A state transition matrix
    int len = sim_node.getLen("parameters");
    A.resize(len,len);
    for ( int i = 0; i < len; i++ ) {
        for ( int j = 0; j < len; j++ ) {
            A(i,j) = sim_node.getDouble("A", i*len+j);
        }
    }
    
    printf("A state transition matrix: %d x %d\n", len, len);
    // for ( int i = 0; i < len; i++ ) {
    //     printf("  ");
    //     for ( int j = 0; j < len; j++ ) {
    //         printf("%.3f ", A(i,j));
    //     }
    //     printf("\n");
    // }
    hal.scheduler->delay(200);
}

void sim_mgr_t::update() {
}
