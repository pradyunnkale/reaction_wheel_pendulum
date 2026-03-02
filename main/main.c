// TODO: ADD IFDEF FOR DIFFERENT CONTROLLERS (MAKE SURE TO INCLUDE IN COMPILE TAGS)
#include <stdio.h>
#include "pid.h"
#include "dynamics.h"
#include "ekf.h"
#include "measurement.h"
#include "state.h"

const float dt = 0.01;

const dynamics_params_t params = {
  .mw = 1,
  .mr = 1,
  .l = 1,
  .g = 9.81,
  .r = 0.1,
  .Iw = 0.5f * 1 * 0.1 * 0.1
};

pid_t pidc = {
  .Kp = 50,
  .Ki = 1,
  .Kd = 50,
  .u_max = 50,
  .integral = 0,
  .prev_err = 0,
  .dt = dt
};

void app_main(void)
{
  state_t xk = {
    .theta = 0,
    .dtheta = 0,
    .phi = 0,
    .dphi = 0
  };

  float uk = 0;

  ekf_t ekf = {0};
  for (int i = 0; i < 4; i++)
    ekf.P[i][i] = 1;

  for (int i = 0; i < 4; i++)
    ekf.Q[i][i] = 0.000001;

  for (int i = 0; i < 2; i++)
    ekf.R[i][i] = 0.25f;
  
  measurement_t y = {
		.accel_x = 0,
		.gyro_z = 0
	};

  for (;;)
  {
    // y = get_measurement(); // Need to implement, gets the measurement from sensors or hitl test
    ekf_step(f_discrete_wrapper, measurement_model, &ekf, &params, &xk, uk, dt, &y, &xk);
    uk = pid(&pidc, &xk);
  }
}
