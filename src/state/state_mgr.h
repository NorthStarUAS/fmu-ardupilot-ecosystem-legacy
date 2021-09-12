#include "airborne.h"
#include "switches.h"

class state_mgr_t {

private:
    
    airborne_t airborne;
    switches_t switches;

public:

    void init();
    void update();

};

