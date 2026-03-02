#include "measurement.h"
#include <math.h>

void measurement_model(const state_t * x, const dynamics_params_t * params, measurement_t * y)
{
  float g, theta, dtheta, dphi;
  g = params->g;
  theta = x->theta;
  dtheta = x->dtheta;
  dphi = x->dphi;

  y->accel_x = g * sinf(theta);
  y->gyro_z = dtheta + dphi;
}