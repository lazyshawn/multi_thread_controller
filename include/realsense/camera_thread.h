#pragma once

#include "realsense/camera_driver.h"
#include "realsense/shared_variable.h"

#define CAMERA_PERIOD (40000000)

/* 定义在 camera_thread.cpp 中 */
extern ObjState objState;

void camera_controller();

