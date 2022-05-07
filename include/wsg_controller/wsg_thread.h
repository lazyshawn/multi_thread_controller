#pragma once

#include "wsg_controller/wsg_driver.h"
#include "wsg_controller/shared_variable.h"

#define WSG_PERIOD (80000000)

extern WSGConfig wsgConfig;

void wsg_controller();

