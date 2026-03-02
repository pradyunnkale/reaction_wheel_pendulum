#include "rk4.h"

void rk4_step(const continuous_func_t f, const state_t * xk, float uk, float dt, const dynamics_params_t * params,  state_t * xk1)
{
  state_t k1, k2, k3, k4, temp;
  f(xk, uk, params, &k1);

  temp.theta = xk->theta + 0.5f * k1.theta * dt;
	temp.dtheta = xk->dtheta + 0.5f * k1.dtheta * dt;
	temp.phi= xk->phi + 0.5f * k1.phi * dt;
	temp.dphi = xk->dphi + 0.5f * k1.dphi * dt;
	f(&temp, uk, params, &k2);

	temp.theta = xk->theta + 0.5f * k2.theta * dt;
	temp.dtheta = xk->dtheta + 0.5f * k2.dtheta * dt;
	temp.phi= xk->phi + 0.5f * k2.phi * dt;
	temp.dphi = xk->dphi + 0.5f * k2.dphi * dt;
	f(&temp, uk, params, &k3);

	temp.theta = xk->theta + k3.theta * dt;
	temp.dtheta = xk->dtheta + k3.dtheta * dt;
	temp.phi= xk->phi + k3.phi * dt;
	temp.dphi = xk->dphi + k3.dphi * dt;
	f(&temp, uk, params, &k4);

	xk1->theta = xk->theta + (dt / 6.0f) * (k1.theta + 2 * k2.theta + 2 * k3.theta + k4.theta);
	xk1->dtheta = xk->dtheta + (dt / 6.0f) * (k1.dtheta + 2 * k2.dtheta + 2 * k3.dtheta + k4.dtheta);
	xk1->phi = xk->phi + (dt / 6.0f) * (k1.phi + 2 * k2.phi + 2 * k3.phi + k4.phi);
	xk1->dphi = xk->dphi + (dt / 6.0f) * (k1.dphi + 2 * k2.dphi + 2 * k3.dphi + k4.dphi);
}
