#include "ekf.h"
#include "esp_dsp.h"
#include <math.h>

void ekf_step(discrete_dyn_func_t f, measurement_func_t h, ekf_t * ekf, const dynamics_params_t * params, const state_t * xhk, float uk, float dt, const measurement_t *y_meas, state_t *xhk1)
{
  // Initialization
  float theta, dtheta, phi, dphi;
  float mr, mw, g, l, Iw;
  float F[4][4] = {0};
  float H[2][4] = {0};
  float Fk[4][4];
  float P_pred[4][4];
  state_t x_pred;
  measurement_t y_pred;

  mr = params->mr;
  mw = params->mw;
  g = params->g;
  l = params->l;
  Iw = params->Iw;

  theta = xhk->theta;
  dtheta = xhk->dtheta;
  phi = xhk->phi;
  dphi = xhk->dphi;

  // Jacobian of f
  F[0][1] = 1.0f;
  F[1][0] = ((0.5f * mr * g * l + mw * g * l) * cosf(theta)) / ((1.0f / 3.0f) * mr * l * l + mw * l * l);
  F[2][3] = 1.0f;
  F[3][0] = -F[1][0];

  // Jacobian of h
  H[0][0] = g * cosf(theta);
  H[1][1] = 1.0f;
  H[1][3] = 1.0f;

  // Discretize F: Fk = I + F * dt 
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      Fk[i][j] = (i == j ? 1.0f : 0.0f) + F[i][j] * dt;

  // PREDICT STEP
  // x_pred = f(xk, uk)
  f(xhk, uk, dt, params, &x_pred);

  // P_pred = Fk * P * Fk' + Q
  float temp1[4][4], Fk_T[4][4];
  dspm_mat_transpose_f32((float *)Fk, (float *)Fk_T, 4, 4);
  dspm_mult_f32((float *)Fk, (float *)ekf->P, (float *)temp1, 4, 4, 4);
  dspm_mult_f32((float *)temp1, (float *)Fk_T, (float *)P_pred, 4, 4, 4);
  // Add Q
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      P_pred[i][j] += ekf->Q[i][j];
  
  // y_pred = h(x_pred)
  h(&x_pred, params, &y_pred);

  // UPDATE STEP 
  // S = H * P_pred * H'+ R
  float temp2[2][4], H_T[4][2], S[2][2];
  dspm_mat_transpose_f32((float *)H, (float *)H_T, 2, 4);
  dspm_mult_f32((float *)H, (float *)P_pred, (float *)temp2, 2, 4, 4);
  dspm_mult_f32((float *)temp2, (float *)H_T, (float *)S, 2, 4, 2);
  // Add R
  for (int i = 0; i < 2; i++)
    for (int j = 0; j < 2; j++)
      S[i][j] += ekf->R[i][j];

  // K = P_pred * H' * inv(S)
  float S_inv[2][2], temp3[4][2], K[4][2];
  dsps_mat_inv_f32((float *)S, (float *)S_inv, 2);
  dspm_mult_f32((float *)P_pred, (float *)H_T, (float *)temp3, 4, 4, 2);
  dspm_mult_f32((float *)temp3, (float *)S_inv, (float *)K, 4, 2, 2);

  // Innovation: yinno = y_meas - y_pred
  float yinno[2];
  yinno[0] = y_meas->accel_x - y_pred.accel_x;
  yinno[1] = y_meas->gyro_z - y_pred.gyro_z;

  // x_new = x_pred + K * yinno
  float K_yinno[4];
  dspm_mult_f32((float *)K, yinno, K_yinno, 4, 2, 1);
  xhk1->theta = x_pred.theta + K_yinno[0];
  xhk1->dtheta = x_pred.dtheta + K_yinno[1];
  xhk1->phi = x_pred.phi + K_yinno[2];
  xhk1->dphi = x_pred.dphi + K_yinno[3];

  // P_new = (I - K * H) * P_pred;
  float KH[4][4], I_KH[4][4];
  dspm_mult_f32((float *)K, (float *)H, (float *)KH, 4, 2, 4);
  for (int i = 0; i < 4; i++)
    for (int j = 0; j < 4; j++)
      I_KH[i][j] = (i == j ? 1.0f : 0.0f) - KH[i][j];
  dspm_mult_f32((float *)I_KH, (float *)P_pred, (float *)ekf->P, 4, 4, 4);
  return;
}