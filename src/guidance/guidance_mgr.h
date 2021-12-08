#include "props2.h"

#include "circle_mgr.h"
#include "route_mgr.h"

class guidance_mgr_t {
    
public:
    void init();
    void update( float dt );

private:
    PropertyNode nav_node;

    circle_mgr_t circle_mgr;
    route_mgr_t route_mgr;
};
