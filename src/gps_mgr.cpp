#include <AP_HAL/AP_HAL.h>
#include <AP_RTC/AP_RTC.h>

#include <AP_SerialManager/AP_SerialManager.h>

#include "setup_board.h"

#include "time.h"

#include "nav/coremag.h"
#include "nav/nav_constants.h"
#include "gps_mgr.h"


// fake gcs object needed and created in top level FMU code.

void gps_mgr_t::setup() {
    gps_node = PropertyNode("/sensors/gps");
    
    // Initialize the UART for GPS system
    //    serial_manager.init();
    gps.init(serial_manager);
}

void gps_mgr_t::update() {
    gps.update();               // AP_GPS
    if (last_message_ms != gps.last_message_time_ms()) {
        last_message_ms = gps.last_message_time_ms();
        gps_millis = gps.last_message_time_ms();
        unix_usec = gps.time_epoch_usec();
        if ( unix_usec > last_unix_usec ) {
            AP::rtc().set_utc_usec(unix_usec, AP_RTC::SOURCE_GPS);
            last_unix_usec = unix_usec;
        }
        double unix_sec = gps.time_epoch_usec() / 1000000.0;
        if ( gps.status() >= AP_GPS::GPS_Status::GPS_OK_FIX_3D ) {
            if ( !gps_acquired ) {
                // first 3d fix
                gps_acquired = true;
                gps_settle_timer = AP_HAL::millis();
                console->printf("GPS: 3d fix acquired.\n");
                console->printf("GPS: unix time = %d\n", (unsigned int)(gps.time_epoch_usec() / 1000000.0));
                update_magvar(unix_sec);
                console->printf("Local magvar (deg) = %.1f\n", magvar_rad*R2D);
            } else if ( gps_acquired and !gps_settled ) {
                if ( AP_HAL::millis() - gps_settle_timer > 10000 ) {
                    // 10 seconds
                    console->printf("GPS: settled for 10 seconds.\n");
                    gps_settled = true;
                    gps_node.setBool("settle", gps_settled);
                }
            }
        } else {
            // less than 3d fix
            if ( gps_acquired and !gps_settled ) {
                // unaquire if we lose fix before settling
                gps_acquired = false;
                console->printf("lost fix before settling, unaquire gps\n");
            }
            
        }

        // get time and date
        struct tm *tm;
        uint64_t time_usec;
        if ( ! AP::rtc().get_utc_usec(time_usec) ) {
            console->printf("RTC clock not yet set!\n");
            time_usec = gps.time_epoch_usec();
        }
        // generate broken-down time
        time_t time_sec = time_usec / 1000000U;
        tm = gmtime(&time_sec);
        
        // publish
        gps_node.setUInt("millis", gps_millis);
        gps_node.setDouble("timestamp", gps_millis / 1000.0);
        gps_node.setUInt64("unix_usec",  time_usec);
        const Location &loc = gps.location();
        gps_node.setInt("latitude_raw", loc.lat);
        gps_node.setInt("longitude_raw", loc.lng);
        gps_node.setDouble("latitude_deg", loc.lat / 10000000.0l);
        gps_node.setDouble("longitude_deg", loc.lng / 10000000.0l);
        gps_node.setFloat("altitude_m", loc.alt / 100.0);
        const Vector3f vel = gps.velocity();
        gps_node.setFloat("vn_mps", vel.x);
        gps_node.setFloat("ve_mps", vel.y);
        gps_node.setFloat("vd_mps", vel.z);
        gps_node.setInt("satellites", gps.num_sats());
        gps_node.setInt("status", gps.status());
        float hacc = 0.0;
        float vacc = 0.0;
        gps.horizontal_accuracy(hacc);
        gps.vertical_accuracy(vacc);
        gps_node.setFloat("horiz_accuracy_m", hacc);
        gps_node.setFloat("vertical_accuracy_m", vacc);
        gps_node.setFloat("hdop", gps.get_hdop() / 100.0);
        gps_node.setFloat("vdop", gps.get_vdop() / 100.0);
        gps_node.setInt("year", tm->tm_year + 1900);
        gps_node.setInt("month", tm->tm_mon + 1);
        gps_node.setInt("day", tm->tm_mday);
        gps_node.setInt("hour", tm->tm_hour);
        gps_node.setInt("min", tm->tm_min);
        gps_node.setInt("sec", tm->tm_sec);
        // gps_node.pretty_print();
    }
}

void gps_mgr_t::update_magvar( time_t unix_sec ) {
    long int jd = unixdate_to_julian_days( unix_sec );
    console->printf("GPS: julian days = %ld\n", jd);
    const Location &loc = gps.location();
    double lat_rad = (loc.lat / 10000000.0) * D2R;
    double lon_rad = (loc.lng / 10000000.0) * D2R;
    float alt_m = loc.alt / 100.0;
    double fields[6];
    console->printf("gps loc = %.8f %.8f\n", loc.lat / 10000000.0, loc.lng / 10000000.0);
    magvar_rad = calc_magvar( lat_rad, lon_rad, alt_m / 1000.0, jd, fields );
    mag_ned(0) = fields[3];
    mag_ned(1) = fields[4];
    mag_ned(2) = fields[5];
    mag_ned.normalize();
    console->printf("GPS: ideal mag vector = %.3f %.3f %.3f\n",
                    mag_ned(0), mag_ned(1), mag_ned(2));
}
