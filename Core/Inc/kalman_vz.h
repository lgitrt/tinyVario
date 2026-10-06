/*
 * kalman_vz.h — Kalman filter: altitude, vertical speed and accel bias
 *
 * Author: Luca Obwegs
 *
 * State vector  x = [h, vz, az_bias]
 *   h        : altitude above reference [m], up positive
 *   vz       : vertical speed [m/s], up positive
 *   az_bias  : world-frame vertical accel bias [m/s^2]
 *
 * Prediction at every firmware IMU tick (26 Hz):  x' = F*x + B*u,  P' = F*P*F' + Q
 * Update when baro_new == 1 (~8 Hz):     Kalman gain + Joseph-form P update
 *
 * Input az_world is the NED-down specific force from Madgwick [m/s^2].
 * At rest az_world ≈ -g; the filter converts it to a_up = -(az_world + g).
 */
#ifndef INC_KALMAN_VZ_H_
#define INC_KALMAN_VZ_H_

#include <stdint.h>        /* uint8_t */
#include "filter_tuning.h" /* KF_Q_AZ, KF_R_H, KF_GATE, KF_P0_*, KF_P0_ZUPT_CROSS */

/* alt0_m: initial altitude above reference [m] — pass 0.0f at power-on */
void KalmanVz_Init(float alt0_m);
void KalmanVz_ZeroVelocity(void);  /* ZUPT: zero vz state and reset velocity covariance */
/* Override the gravity constant used for u = -(az_world + g).
   Call after calibration with the measured |g| to cancel accel Z-bias. */
void KalmanVz_SetGravity(float g_meas);

/* Call once per IMU interrupt.
 *   az_world   : NED-down specific force from Madgwick [m/s^2]
 *   alt_baro_m : barometer altitude above reference [m]
 *   baro_new   : 1 when a fresh baro sample just arrived, 0 otherwise
 *   vz_out     : estimated vertical speed, up positive [m/s]
 *   alt_out    : estimated altitude [m]                               */
void KalmanVz_Update(float az_world, float alt_baro_m, uint8_t baro_new,
                     float *vz_out, float *alt_out);

#endif /* INC_KALMAN_VZ_H_ */
