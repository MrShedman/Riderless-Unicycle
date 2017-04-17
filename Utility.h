#pragma once

#include "Arduino.h"

float mapf(float x, float in_min, float in_max, float out_min, float out_max);

int32_t applyDeadband(int32_t value, int32_t deadband);

float applyDeadbandf(float value, float deadband);