#pragma once

#include "wsg_controller/wsg_driver.h"
#include "wsg_controller/shared_variable.h"

#define WSG_PERIOD (80000000)
#define WSG_SERVO_TIME (double)WSG_PERIOD/NSEC_PER_SEC

extern WSGConfig wsgConfig;

void wsg_controller();

