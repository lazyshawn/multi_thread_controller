#pragma once

#include "ur5e_controller/ur5e_kinematics.h"
#include "ur5e_controller/shared_variable.h"

#define UR_PERIOD (8000000)
#define UR_SERVO_TIME (double)UR_PERIOD/NSEC_PER_SEC

extern Ur5eShared ur5eShared;

void ur5e_controller();

