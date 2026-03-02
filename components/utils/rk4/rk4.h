#pragma once

#include "dynamics.h"

typedef void (*continuous_func_t)(const state_t* x, float u, const dynamics_params_t * params, state_t* dxdt);

void rk4_step(const continuous_func_t f, const state_t* xk, float uk, float dt, const dynamics_params_t * params, state_t * xk1);
