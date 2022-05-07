#pragma once

#include "uskin/sensor_driver.h"
#include "uskin/shared_variable.h"

#define USKIN_PERIOD (80000000)

extern Force force;

void uskin_controller();

