#!/usr/bin/env python3

import math
import numpy as np

import navpy            # pip install git+https://github.com/NavPy/NavPy.git
from rcUAS import wgs84 # Rice Creek UAS: rc-flight

import transformations

r2d = 180.0 / math.pi
d2r = math.pi / 180.0

euler_deg = [45, -30, 310]
pos_lla = [ 45.0, -93.0, 300.0 ]
tgt_lla = [ 45.001, -93.001, 287.0 ]
ref_lla = pos_lla

def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    else:
        return v / norm

# Computes a quaternion from the given euler angles  
def eul2quat(phi, the, psi):
    sin_psi = math.sin(psi*0.5)
    cos_psi = math.cos(psi*0.5)
    sin_the = math.sin(the*0.5)
    cos_the = math.cos(the*0.5)
    sin_phi = math.sin(phi*0.5)
    cos_phi = math.cos(phi*0.5)
    q = np.array( [cos_psi*cos_the*cos_phi + sin_psi*sin_the*sin_phi,
                   cos_psi*cos_the*sin_phi - sin_psi*sin_the*cos_phi,
                   cos_psi*sin_the*cos_phi + sin_psi*cos_the*sin_phi,
                   sin_psi*cos_the*cos_phi - cos_psi*sin_the*sin_phi] )
    return q

# Quaternion to C_N2B
def quat2dcm(q):
    C_N2B = np.zeros((3,3))
    q0 = q[0]; q1 = q[1]; q2 = q[2]; q3 = q[3]

    C_N2B[0,0] = 2*(q0*q0 + q1*q1) - 1
    C_N2B[1,1] = 2*(q0*q0 + q2*q2) - 1
    C_N2B[2,2] = 2*(q0*q0 + q3*q3) - 1

    C_N2B[0,1] = 2*(q1*q2 + q0*q3)
    C_N2B[0,2] = 2*(q1*q3 - q0*q2)
	
    C_N2B[1,0] = 2*(q1*q2 - q0*q3)
    C_N2B[1,2] = 2*(q2*q3 + q0*q1)
	
    C_N2B[2,0] = 2*(q1*q3 + q0*q2)
    C_N2B[2,1] = 2*(q2*q3 - q0*q1)
	
    return C_N2B

# compute wgs84 distance/course over ground
(course, reverse_course, dist_m) = wgs84.geo_inverse( pos_lla[0], pos_lla[1],
                                                      tgt_lla[0], tgt_lla[1] )
print("from (lla):", pos_lla)
print("to (lla):", tgt_lla)
print("(lla) course: %.2f  dist(m): %.2f" % (course, dist_m) )

# compute ned distance/course over ground
pos_ned = navpy.lla2ned( pos_lla[0], pos_lla[1], pos_lla[2],
                         ref_lla[0], ref_lla[1], 0.0 )
tgt_ned = navpy.lla2ned( tgt_lla[0], tgt_lla[1], tgt_lla[2],
                         ref_lla[0], ref_lla[1], 0.0 )
print("from (ned):", pos_ned)
print("to (ned):", tgt_ned)

course = math.atan2(tgt_ned[1] - pos_ned[1], tgt_ned[0] - pos_ned[0]) * r2d
if course < 0: course += 360
dist_m = np.linalg.norm(pos_ned[:2] - tgt_ned[:2])
print("(ned) course: %.2f  dist(m): %.2f" % (course, dist_m) )

# compute azimuth (from ned)
az_deg = math.atan2(pos_ned[2] - tgt_ned[2], dist_m) * r2d
print("(ned) azimuth: %.2f\n" % az_deg)

#body2ned = transformations.quaternion_from_euler(euler_deg[2]*d2r,
#                                                 euler_deg[1]*d2r,
#                                                 euler_deg[0]*d2r,
#                                                 'rzyx')
ned2body = eul2quat(euler_deg[0]*d2r, euler_deg[1]*d2r, euler_deg[2]*d2r)
print(ned2body)

C_N2B = quat2dcm(ned2body)
C_B2N = np.linalg.inv(C_N2B)
print("C_N2B:\n", C_N2B)
print("C_B2N:\n", C_B2N)

nose_vec = np.array( [1, 0, 0] )
mount_vec = np.array( [0, 1, 0] )

#t = transformations.quaternion_transform(body2ned, nose_vec)
t = C_B2N @ mount_vec
print("t (mount):", t)

point_vec_ned = normalize(tgt_ned - pos_ned)
print("pointing vector (ned):", point_vec_ned)
t = C_N2B @ point_vec_ned
print("point vector (in body frame):", t)
print("norm(t):", normalize(t))

pan = 90 - math.atan2(t[0], t[1]) * r2d
print("pan: %.2f\n" % pan)

length = np.linalg.norm( [t[0], t[1] ] )
print("length:", length)
tilt = math.atan2(-t[2], length) * r2d
print("tilt: %.2f\n" % tilt)
