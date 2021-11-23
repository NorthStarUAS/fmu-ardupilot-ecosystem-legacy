#pragma once

#include <AP_HAL/AP_HAL.h>

#include <math.h>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"

Eigen::Vector3f pointing_update(Eigen::Vector3d pos_lla,
				Eigen::Vector3f euler_deg,
				Eigen::Vector3d tgt_lla);
