#pragma once

#include "miniros/miniros.h"
#include "wsg_controller/wsg_thread.h"
#include "uskin/uskin_thread.h"
#include "ur5e_controller/ur5e_interface.h"
#include "realsense/camera_thread.h"

void devices_home();
void pick_and_place(double hoverHeight);

