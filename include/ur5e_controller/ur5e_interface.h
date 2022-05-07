#pragma once

#include "ur5e_controller/ur5e_thread.h"

typedef Eigen::Matrix<double,6,1> Vec6d;
typedef Eigen::Matrix<double,4,4> Mat4d;
typedef Eigen::Matrix<double,3,3> Mat3d;
typedef Eigen::Matrix<double,6,6> Mat6d;

namespace ur5e {
void go_to_pose(Mat4d tran, double time);
void go_to_joint(THETA joint, double time);
void go_home(double time);
void plane_pivot();
void plane_screw();
void move_ref_to_end();
} // namespace ur5e
