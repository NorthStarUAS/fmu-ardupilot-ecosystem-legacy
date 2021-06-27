#include "props2.h"

class menu_t {
    
private:
    uint32_t reboot_count = 0;
    const char *reboot_cmd = "reboot";
    
public:
    bool display_pilot = false;
    bool display_gps = false;
    bool display_airdata = false;
    bool display_imu = false;
    bool display_nav = false;
    bool display_act = false;
        
    void display();
    void update();
};
