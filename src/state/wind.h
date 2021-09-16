#pragma once

#include "props2.h"
#include "util/lowpass.h"

class wind_est_t {
    
private:
    
    PropertyNode airdata_node;
    PropertyNode filter_node;
    PropertyNode orient_node;
    PropertyNode vel_node;
    PropertyNode wind_node;
    
    rcLowPassFilter we_filt;
    rcLowPassFilter wn_filt;
    rcLowPassFilter pitot_scale_filt;

public:
    
    void init();
    void update( double dt );
    
};
