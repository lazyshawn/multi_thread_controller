#pragma once

#include "master/master_shared.h"
#include "wsg_controller/wsg_thread.h"
#include "ur5e_controller/ur5e_interface.h"
#include "realsense/camera_thread.h"
#include "dydw/dydw_thread.h"

enum class Task{idle, press, init_grasp, pivot, rotate, calib};

void planner_controller();
void task_press();
void pivot_finger(double x, double z, double dq);
void task_rotate(double x, double z, double dq);
void actual_force(std::vector<double>& force, double theta);
void planner2ur(THETA curJoint, THETA goalJoint);

