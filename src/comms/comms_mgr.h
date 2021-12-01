// manage message-based comms and relays

#pragma once

#include "props2.h"
#include "util/ratelimiter.h"

#include "info.h"
#include "menu.h"
#include "message_link.h"

class comms_mgr_t {  
private:
    message_link_t gcs_link;
    message_link_t host_link;
    info_t info;
    menu_t menu;
    
    RateLimiter info_timer;
    RateLimiter heartbeat;
    uint32_t tempTimer;
    uint32_t counter;

    PropertyNode config_node;
    PropertyNode imu_node;
    PropertyNode status_node;
    
public:
    void init();
    void update();
};
