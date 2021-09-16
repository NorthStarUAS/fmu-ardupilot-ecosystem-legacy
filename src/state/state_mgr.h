#include "airdata_helper.h"
#include "ground.h"
#include "switches.h"

class state_mgr_t {

private:
    
    airdata_helper_t airdata;
    ground_est_t ground;
    switches_t switches;

public:

    void init();
    void update(float dt);

};
