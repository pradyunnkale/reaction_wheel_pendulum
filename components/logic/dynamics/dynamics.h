#pragma once

#include "state.h"

typedef struct
{
  float mw;
  float mr;
  float l;
  float Iw;
  float g;
  float r;
}
dynamics_params_t;

void nonlinear_continuous(const state_t * x, const float u, 
                          const dynamics_params_t * params, 
                          state_t * dxdt);
