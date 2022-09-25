#pragma once

#include "master/master_shared.h"
#include "wsg_controller/wsg_thread.h"
#include "uskin/uskin_thread.h"
#include "ur5e_controller/ur5e_interface.h"
#include "realsense/camera_thread.h"
#include "dydw/dydw_thread.h"

#define HFVC_PERIOD (40000000)

extern HfvcShared hfvcShared;

// 求解器标志位
enum class PlannerFlag {
  hfvc, interface
};

void hfvc_controller();
Arr3d hfvc_executer(std::vector<double> hfvcCmd, Arr3d state, std::vector<double> force);

