#include <stdio.h>

#include "nav/nav_functions.h"
#include "point.h"

const double r2d = 180.0l / M_PI;
const double d2r = M_PI / 180.0l;

static Eigen::Vector3f lla2ned( Eigen::Vector3d lla, Eigen::Vector3d ref ) {
    Eigen::Vector3d ecef  = lla2ecef(lla);
    Eigen::Vector3d ecef0 = lla2ecef(ref);
    Eigen::Vector3f ned  = ecef2ned(ecef-ecef0, ref);
    return ned;
}

static void printvec( const char *label, Eigen::Vector3f v ) {
    printf("%s %.2f %.2f %.2f\n", label, v(0), v(1), v(2));
}
static void printquat( const char *label, Eigen::Quaternionf q ) {
    printf("%s %.2f %.2f %.2f %.2f\n", label, q.w(), q.x(), q.y(), q.z());
}
static void printmat3( const char *label, Eigen::Matrix3f m ) {
    printf("%s:\n", label);
    printf("  %.2f %.2f %.2f %.2f\n", m(0,0), m(0,1), m(0,2));
    printf("  %.2f %.2f %.2f %.2f\n", m(1,0), m(1,1), m(1,2));
    printf("  %.2f %.2f %.2f %.2f\n", m(2,0), m(2,1), m(2,2));
}

Eigen::Vector3f pointing_update(Eigen::Vector3d pos_lla, Eigen::Vector3f euler_deg, Eigen::Vector3d tgt_lla) {
    static Eigen::Vector3d ref_lla;
    // pos_lla << 45.0*d2r, -93.0*d2r, 300.0;
    // tgt_lla << 45.001*d2r, -93.001*d2r, 287.0;
    ref_lla = pos_lla;
    ref_lla(2) = 0.0;

    // static Eigen::Vector3f euler_deg;
    // euler_deg << 45, -30, 310;
    
    // Where to point in ned space
    Eigen::Vector3f pos_ned = lla2ned( pos_lla, ref_lla );
    Eigen::Vector3f tgt_ned = lla2ned( tgt_lla, ref_lla );
    printvec("from (ned)", pos_ned);
    printvec("to (ned)", tgt_ned);

    /*
    float course_deg = atan2(tgt_ned(1)-pos_ned(1), tgt_ned(0)-pos_ned(0)) * r2d;
    if (course_deg < 0) { course_deg += 360.0; }
    
    float dist_m = (pos_ned.head(2) - tgt_ned.head(2)).norm();
    printf("(ned) course: %.2f  dist(m): %.2f", course_deg, dist_m);
    
    float az_deg = atan2(pos_ned(2)-tgt_ned(2), dist_m) * r2d;
    printf("(ned) azimuth: %.2f\n", az_deg);
    */
    
    // construct body to ned transformation matrix
    Eigen::Quaternionf ned2body = eul2quat(euler_deg(0)*d2r, euler_deg(1)*d2r, euler_deg(2)*d2r);
    printquat("ned2body", ned2body);

    Eigen::Matrix3f C_N2B = quat2dcm(ned2body);
    printmat3("C_N2B", C_N2B);
    // Eigen::Matrix3f C_B2N = C_N2B.inverse();
    // printmat3("C_B2N", C_B2N);

    Eigen::Vector3f mount_vec;
    mount_vec << 0.0, 1.0, 0.0;
    
    Eigen::Vector3f point_vec_ned = tgt_ned - pos_ned;
    point_vec_ned.normalize();

    Eigen::Vector3f t = C_N2B * point_vec_ned;
    printvec("point vector (in body frame)", t);

    float pan = 90.0 - atan2(t(0), t(1)) * r2d;
    printf("pan: %.2f\n", pan);
    float length = t.head(2).norm();
    printf("length: %.2f\n", length);
    float tilt = atan2(-t(2), length) * r2d;
    printf("tilt: %.2f\n", tilt);

    Eigen::Vector3f result;
    result << pan, tilt, 0.0;
    return result;
}
