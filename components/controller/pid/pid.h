#pragma once

#include "state.h"

typedef struct
{
  float Kp;
  float Ki;
  float Kd;
  float u_max;
  float integral;
  float prev_err;
  float dt;
}
pid_t;

// Returns the control output
float pid(pid_t * pid, state_t x);