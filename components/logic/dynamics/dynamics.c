#include "dynamics.h"
#include <math.h>

void nonlinear_continuous(const state_t * x, const float u, const dynamics_params_t * params, state_t * dxdt)
{
  float theta, dtheta, dphi, mr, mw, l, Iw, g;
  mr = params->mr;
  mw = params->mw;
  l = params->l;
  Iw = params->Iw;
  g = params->g;
  theta = x->theta;
  dtheta = x->dtheta;
  dphi = x->dphi;

  dxdt->theta = dtheta;
  dxdt->dtheta = ((0.5f * mr * g * l + mw * g * l) * sinf(theta) - u) / ((1.0f / 3.0f) * mr * l * l + mw * l * l);
  dxdt->phi = dphi;
  dxdt->dphi = u / Iw - dxdt->dtheta;
  return;
}