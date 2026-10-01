/*
 * madgwick.h — Madgwick AHRS filter, NED / body-Z-down convention
 *
 * Author: Luca Obwegs
 *
 * Body frame : X=forward, Y=right, Z=DOWN  (LSM6DS3 default mount)
 * World frame: NED — North(X), East(Y), Down(Z)
 * At rest    : accel reads [0, 0, -g] m/s^2  (upward reaction force)
 * az_world   : NED-down specific force — at rest ≈ -g m/s^2
 *
 * Parameters from param_init.m, tuned for LSM6DS3 @ 52 Hz.
 */
#ifndef INC_MADGWICK_H_
#define INC_MADGWICK_H_

#include "filter_tuning.h"  /* MADGWICK_BETA, MADGWICK_ZETA, MADGWICK_ACCEL_GATE_FRAC */

/* dt must match SAMPLE_HZ in vario.h (LSM6DS3 ODR = 26 Hz) */
#define MADGWICK_DT    (1.0f / 26.0f)

void  Madgwick_Init(void);
/* Initialise quaternion from a gravity reading so az_world is correct immediately.
   ax/ay/az are the raw accelerometer values in m/s^2 (need not be normalised). */
void  Madgwick_Init_FromAccel(float ax, float ay, float az);
/* ax/ay/az in m/s^2, gx/gy/gz in rad/s. Returns az_world [m/s^2]. */
float Madgwick_Update(float ax, float ay, float az,
                      float gx, float gy, float gz);

#endif /* INC_MADGWICK_H_ */
