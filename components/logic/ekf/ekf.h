#pragma once

#include "state.h"
#include "dynamics.h"
#include "measurement.h"
#include "rk4.h"

typedef struct
{
  float P[4][4];
  float Q[4][4];
  float R[2][2];
}
ekf_t;

// Discrete dynamics model x_{k+1} = f(x_k, u_k, params)
typedef void (*discrete_dyn_func_t)(const state_t *xk,
                                    float uk, 
                                    float dt,
                                    const dynamics_params_t * params,
                                    state_t *xk_1);
// Discrete measurement model
typedef void (*measurement_func_t)(const state_t * x,
                                  const dynamics_params_t * params,
                                  measurement_t * y);

void ekf_step(discrete_dyn_func_t f, 
              measurement_func_t h,
              ekf_t * ekf,
              const dynamics_params_t * params,
              const state_t * xhk,
              float uk,
              float dt,
              const measurement_t *y_meas,
              state_t *x_hat);

void f_discrete_wrapper(const state_t* xk, float uk, float dt, const dynamics_params_t * params, state_t * xk1);
