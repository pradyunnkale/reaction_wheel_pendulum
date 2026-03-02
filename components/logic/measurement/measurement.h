#pragma once

#include "state.h"
#include "dynamics.h"

typedef struct
{
  float accel_x;
  float gyro_z;
}
measurement_t;

void measurement_model(const state_t * x, const dynamics_params_t * params, measurement_t * y);