#pragma once

#include "setup_board.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

bool affine_from_points(Eigen::MatrixXf src, Eigen::MatrixXf dst, bool shear, bool scale, Eigen::MatrixXf &M );
