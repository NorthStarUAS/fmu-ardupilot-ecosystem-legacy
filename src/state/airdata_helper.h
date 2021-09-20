#include "props2.h"

class airdata_helper_t {
    
private:
    
    PropertyNode airdata_node;
    PropertyNode pos_node;
    PropertyNode vel_node;

    // is airborne
    float up_kts = 12.0;
    float down_kts = 8.0;
    float up_m = 6;
    float down_m = 3;
    bool is_airborne = false;

    // flight timer
    uint32_t last_millis = 0;
    uint32_t flight_millis = 0;

public:

    void init();
    void update();
    
};
