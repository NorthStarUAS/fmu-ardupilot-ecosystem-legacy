#include "props2.h"

class airborne_t {
    
private:
    
    PropertyNode status_node;
    PropertyNode pos_node;
    PropertyNode vel_node;
    
    float up_kts = 12.0;
    float down_kts = 8.0;
    float up_ft = 25;
    float down_ft = 10;
    bool is_airborne = false;

    uint32_t last_millis = 0;
    float flight_secs = 0.0;

public:

    void init();
    void update();
    
};
