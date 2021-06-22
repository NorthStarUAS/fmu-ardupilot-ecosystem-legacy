#include <AP_HAL/AP_HAL.h>

#include <AP_SerialManager/AP_SerialManager.h>

// maybe we need these?
#include <GCS_MAVLink/GCS_Dummy.h>

#include "setup_board.h"

#include "nav/coremag.h"
#include "nav/nav_constants.h"
#include "gps_mgr.h"

// Serial manager is needed for UART communications
static AP_SerialManager serial_manager;

// create fake gcs object (need for the AP gps driver)
GCS_Dummy _gcs;
const AP_Param::GroupInfo GCS_MAVLINK_Parameters::var_info[] = {
    AP_GROUPEND
};

void gps_mgr_t::setup() {
    gps_node = PropertyNode("/sensors/gps");
    
    // Initialize the UART for GPS system
    serial_manager.init();
    gps.init(serial_manager);
}

void gps_mgr_t::update() {
    // suck in any available gps bytes
    gps.update();
    if (last_message_ms != gps.last_message_time_ms()) {
        last_message_ms = gps.last_message_time_ms();
        gps_millis = gps.last_message_time_ms();
        unix_sec = gps.time_epoch_usec() / 1000000.0;
        // update_unix_sec();
        if ( gps.status() >= AP_GPS::GPS_Status::GPS_OK_FIX_3D ) {
            if ( !gps_acquired ) {
                // first 3d fix
                gps_acquired = true;
                gps_settle_timer = AP_HAL::millis();
                update_magvar();
                console->printf("GPS: 3d fix acquired.\n");
                console->printf("GPS: unix time = %ld\n", (uint32_t)(gps.time_epoch_usec() / 1000000.0));
                console->printf("Local magvar (deg) = %.1f\n", magvar_rad*R2D);
            } else if ( gps_acquired and !gps_settled ) {
                if ( AP_HAL::millis() - gps_settle_timer > 10000 ) {
                    // 10 seconds
                    gps_settled = true;
                }
            }
        }
        
        // publish
        gps_node.setUInt("millis", gps_millis);
        gps_node.setFloat("timestamp", gps_millis / 1000.0);
        gps_node.setFloat("unix_sec", unix_sec);
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
        gps_node.setBool("settle", gps_settled);
    }
}

// void gps_mgr_t::update_unix_sec() {
//     tmElements_t tm;
//     int yr = gps_data.year;
//     if (yr > 99) {
//         yr = yr - 1970;
//     } else {
//         yr += 30;
//     }
//     tm.Year = yr;
//     tm.Month = gps_data.month;
//     tm.Day = gps_data.day;
//     tm.Hour = gps_data.hour;
//     tm.Minute = gps_data.min;
//     tm.Second = gps_data.sec;
//     unix_sec = makeTime(tm);
// }

void gps_mgr_t::update_magvar() {
    long int jd = unixdate_to_julian_days( unix_sec );
    console->printf("GPS: julian days = %ld\n", jd);
    const Location &loc = gps.location();
    double lat_rad = (loc.lat / 10000000.0) * D2R;
    double lon_rad = (loc.lng / 10000000.0) * D2R;
    float alt_m = loc.alt / 100.0;
    double fields[6];
    magvar_rad = calc_magvar( lat_rad, lon_rad, alt_m / 1000.0, jd, fields );
    mag_ned(0) = fields[3];
    mag_ned(1) = fields[4];
    mag_ned(2) = fields[5];
    mag_ned.normalize();
    console->printf("GPS: ideal mag vector = %.3f %.3f %.3f\n",
                    mag_ned(0), mag_ned(1), mag_ned(2));
}
