#include <vector>
using std::vector;

#include "props2.h"

#include "waypoint.h"

class route_mgr_t {
    
public:
    void init();
    bool build( PropertyNode config_node );
    bool build_str( string request );
    void swap();
    waypoint_t get_current_wp();
    waypoint_t get_previous_wp();
    void increment_wp();
    void dribble( bool reset=false );
    void reposition( bool force=false );
    float get_remaining_distance_from_next_waypoint();
    float wind_heading_error( float current_crs_deg, float target_crs_deg );
    void update( float dt );

private:
    PropertyNode route_node;
    PropertyNode pos_node;
    PropertyNode vel_node;
    PropertyNode orient_node;
    PropertyNode home_node;
    PropertyNode active_route_node;
    PropertyNode L1_node;
    PropertyNode targets_node;
    PropertyNode gps_node;
    PropertyNode comms_node;
    PropertyNode wind_node;

    vector<waypoint_t> active_route;
    vector<waypoint_t> standby_route;
    int current_wp = 0;
    bool acquired = false;

    double last_lon = 0.0;
    double last_lat = 0.0;
    float last_az = 0.0;
    uint16_t wp_counter = 0;    // for dribble
    bool dist_valid = false;
};
