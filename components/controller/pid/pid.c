#include "pid.h"

float pid(pid_t * pid, state_t * x)
{
  // Reference is x(1) == 0, where x(1) = theta
  float error = x->theta;
  float u;

  u = pid->Kp * error + 
      pid->Ki * pid->integral + 
      pid->Kd * (error - pid->prev_err);

  pid->integral += error * pid->dt;
  pid->prev_err = error;

  return u;
}