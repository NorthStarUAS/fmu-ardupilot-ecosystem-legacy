#include "setup_board.h"

#include "menu.h"

void menu_t::display() {
    console->printf("%s\n",
                    "  1) Pilot input\n"
                    "  2) GPS\n"
                    "  3) Airdata\n"
                    "  4) IMU\n"
                    "  5) Nav/EFK\n"
                    "  6) Actuator output\n"
                    "  Reboot: type \"reboot\"\n");
}

void menu_t::update() {
    // uint8_t buf[128];
    int16_t user_input = 0;
    // int avail = console->available();
    // if ( avail > 1 ) {
    //     int read = console->read(buf, 128);
    //     for ( int i = 0; i < read; i++ ) {
    //         console->printf("  read: %d %c (%d)\n", i, buf[i], buf[i]);
    //     }
    //     // this is probably a brittle hack
    //     if ( buf[34] == 'r' and buf[35] == 'e' and buf[36] == 'b' and buf[37] == 'o' and buf[38] == 'o' and buf[39] == 't' ) {
    //         console->printf("rebooting for firmward load request ...\n");
    //         hal.scheduler->delay(500);
    //         hal.scheduler->reboot(false);
    //     }
    // } else 
    if ( console->available() ) {
        user_input = console->read();
        printf("read character: %c\n", (char)user_input);
        if ( user_input == '1' ) {
            display_pilot = !display_pilot;
        } else if ( user_input == '2' ) {
            display_gps = !display_gps;
        } else if ( user_input == '3' ) {
            display_airdata = !display_airdata;
        } else if ( user_input == '4' ) {
            display_imu = !display_imu;
        } else if ( user_input == '5' ) {
            display_nav = !display_nav;
        } else if ( user_input == '6' ) {
            display_act = !display_act;
        } else if ( user_input == reboot_cmd[reboot_count] ) {
            reboot_count++;
            if ( reboot_count == strlen(reboot_cmd) ) {
                console->printf("rebooting by user request ...\n");
                hal.scheduler->delay(250);
                hal.scheduler->reboot(false);
            }
        } else if ( user_input == reboot_cmd[0] ) {
            // allow immediate restart of a failed or partial command
            reboot_count = 1;
        } else {
            reboot_count = 0;
            display();
        }
    }
}
