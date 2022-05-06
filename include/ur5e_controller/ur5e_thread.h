#pragma once

#include "miniros/miniros.h"
#include "ur5e_controller/shared_variable.h"
#include "ur5e_controller/ur5e_interface.h"

#define UR_PERIOD (8000000)
#define UR_SERVO_TIME (double)UR_PERIOD/NSEC_PER_SEC

void ur5e_controller();

