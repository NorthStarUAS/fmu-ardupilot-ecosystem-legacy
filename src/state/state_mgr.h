#include "airdata_helper.h"
#include "switches.h"

class state_mgr_t {

private:
    
    airdata_helper_t airdata;
    switches_t switches;

public:

    void init();
    void update();

};
