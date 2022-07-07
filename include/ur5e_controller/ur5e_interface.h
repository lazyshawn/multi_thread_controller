#pragma once

#include "ur5e_controller/ur5e_thread.h"

typedef Eigen::Matrix<double,6,1> Vec6d;
typedef Eigen::Matrix<double,4,4> Mat4d;
typedef Eigen::Matrix<double,3,3> Mat3d;
typedef Eigen::Matrix<double,6,6> Mat6d;
typedef std::array<double,3> Arr3d;

namespace ur5e {
bool check_ur_state();
/* **************** 空间运动学 **************** */
void go_to_pose(Mat4d tran, double time);
void go_to_joint(THETA joint, double time);
void go_home(double time);
void body_twist(double distance, double time);
/* **************** 平面运动学 **************** */
void tcp_move_2d(Arr3d movement, double time);
void tcp_pivot_2d(Arr3d twist, double time);
void plane_screw(Arr3d screw, double time);
void plane_pivot();
void move_ref_to_end();
void wait_path_clear();
/* **************** 人机交互 **************** */
void print_current_info();
void teleoperate();
} // namespace ur5e
