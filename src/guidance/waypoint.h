#include "props2.h"

class waypoint_t {
public:
    void build( PropertyNode config_node );
    void update_relative_pos( double home_lon_deg, double home_lat_deg,
                              float ref_heading_deg );
    
private:
    bool absolute = true;
    double lon_deg = 0.0;
    double lat_deg = 0.0;
    float hdg_deg = 0.0;
    float dist_m = 0.0;
    float leg_dist_m = 0.0;
};
