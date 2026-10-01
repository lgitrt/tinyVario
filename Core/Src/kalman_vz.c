/*
 * kalman_vz.c — Kalman filter: altitude, vertical speed and accel bias
 *
 * Author: Luca Obwegs
 *
 * Direct C translation of fcn_kalman_vz.m.
 * All matrix constants pre-computed from param_init.m at dt = 1/52 s.
 *
 * State-transition  F = [[1, dt, -dt^2/2],
 *                        [0,  1,      -dt],
 *                        [0,  0,        1]]
 * Control input     B = [dt^2/2, dt, 0]'
 * Measurement       H = [1, 0, 0]   (altitude only)
 *
 * The sparse structure of F and H is exploited throughout to keep the
 * 3x3 linear algebra as a flat set of scalar operations with no loops.
 */
#include "kalman_vz.h"
#include <math.h>

extern volatile uint32_t dbg_kf_gate_reject;

/* Process noise Q, measurement noise R, gate, and initial covariance come
   from filter_tuning.h (included via kalman_vz.h). */
#define KF_G       9.80665f
#define KF_DT      (1.0f / 26.0f)   /* must match SAMPLE_HZ in vario.h  */

/* State-transition F: non-unity off-diagonal elements */
#define KF_F01     (1.0f / 26.0f)                    /*  dt             */
#define KF_F02     (-0.5f / (26.0f * 26.0f))          /* -dt^2/2         */
#define KF_F12     (-1.0f / 26.0f)                    /* -dt             */

/* Control input B */
#define KF_B0      (0.5f / (26.0f * 26.0f))           /*  dt^2/2         */
#define KF_B1      (1.0f / 26.0f)                     /*  dt             */

/* Process noise Q (derived from filter_tuning.h constants and dt = 1/26 s) */
#define KF_Q00     (KF_Q_AZ   / (3.0f * 26.0f * 26.0f * 26.0f))  /* q_az*dt^3/3  */
#define KF_Q01     (KF_Q_AZ   / (2.0f * 26.0f * 26.0f))           /* q_az*dt^2/2  */
#define KF_Q11     (KF_Q_AZ   / 26.0f)                             /* q_az*dt      */
#define KF_Q22     (KF_Q_BIAS / 26.0f)                             /* q_bias*dt    */

/* -----------------------------------------------------------------------
 * Filter state
 * --------------------------------------------------------------------- */
static float x[3];      /* [h, vz, az_bias] */
static float P[3][3];   /* covariance        */
static float kf_g = 9.80665f;  /* calibrated gravity magnitude [m/s^2] */

/* -----------------------------------------------------------------------
 * Public API
 * --------------------------------------------------------------------- */
void KalmanVz_Init(float alt0_m)
{
    x[0] = alt0_m;
    x[1] = 0.0f;
    x[2] = 0.0f;

    P[0][0] = KF_P0_H;  P[0][1] = 0.0f;      P[0][2] = 0.0f;
    P[1][0] = 0.0f;      P[1][1] = KF_P0_VZ;  P[1][2] = 0.0f;
    P[2][0] = 0.0f;      P[2][1] = 0.0f;      P[2][2] = KF_P0_B;
}

void KalmanVz_ZeroVelocity(void)
{
    /* A persistent accel-bias term is exactly what makes the filtered vertical
       speed drift negative while the device is genuinely stationary. Reset the
       bias state together with the speed so the filter does not keep integrating
       the same offset after a ZUPT event. */
    x[1] = 0.0f;
    x[2] = 0.0f;

    /* Small but non-zero covariance keeps the barometer helpful immediately after
       the reset without reintroducing the bias drift mechanism. */
    P[0][1] = P[1][0] = KF_P0_ZUPT_CROSS;
    P[0][2] = P[2][0] = 0.0f;
    P[1][1] = 0.01f;
    P[1][2] = P[2][1] = 0.0f;
    P[2][2] = KF_P0_B;
}

void KalmanVz_SetGravity(float g_meas)
{
    if (g_meas > 8.0f && g_meas < 11.0f)
        kf_g = g_meas;
}

__attribute__((optimize("O3")))
void KalmanVz_Update(float az_world, float alt_baro_m, uint8_t baro_new,
                     float *vz_out, float *alt_out)
{
    /* -------------------------------------------------------------------
     * Control input: vertical acceleration, up positive.
     * az_world (NED-down specific force): at rest ≈ -g.
     * u = -(az_world + g);  bias is removed implicitly by the filter state.
     * ------------------------------------------------------------------- */
    float u = -(az_world + kf_g);

    /* -------------------------------------------------------------------
     * Prediction:  xp = F*x + B*u
     * ------------------------------------------------------------------- */
    float xp0 = x[0] + KF_F01*x[1] + KF_F02*x[2] + KF_B0*u;
    float xp1 = x[1]               + KF_F12*x[2] + KF_B1*u;
    float xp2 = x[2];

    /* -------------------------------------------------------------------
     * Prediction covariance:  Pp = F*P*F' + Q
     * Computed using the sparse structure of F.
     * ------------------------------------------------------------------- */
    /* Temp = F*P  (row by row) */
    float T00 = P[0][0] + KF_F01*P[1][0] + KF_F02*P[2][0];
    float T01 = P[0][1] + KF_F01*P[1][1] + KF_F02*P[2][1];
    float T02 = P[0][2] + KF_F01*P[1][2] + KF_F02*P[2][2];
    float T10 = P[1][0]                   + KF_F12*P[2][0];
    float T11 = P[1][1]                   + KF_F12*P[2][1];
    float T12 = P[1][2]                   + KF_F12*P[2][2];
    float T20 = P[2][0];
    float T21 = P[2][1];
    float T22 = P[2][2];

    /* Pp = Temp*F'  (column by column using F's sparse structure) */
    float Pp00 = T00;
    float Pp01 = KF_F01*T00 + T01;
    float Pp02 = KF_F02*T00 + KF_F12*T01 + T02;
    float Pp10 = T10;
    float Pp11 = KF_F01*T10 + T11;
    float Pp12 = KF_F02*T10 + KF_F12*T11 + T12;
    float Pp20 = T20;
    float Pp21 = KF_F01*T20 + T21;
    float Pp22 = KF_F02*T20 + KF_F12*T21 + T22;

    /* Add Q */
    Pp00 += KF_Q00;
    Pp01 += KF_Q01; Pp10 += KF_Q01;
    Pp11 += KF_Q11;
    Pp22 += KF_Q22;

    /* Symmetrise */
    float s01 = 0.5f*(Pp01+Pp10); Pp01 = s01; Pp10 = s01;
    float s02 = 0.5f*(Pp02+Pp20); Pp02 = s02; Pp20 = s02;
    float s12 = 0.5f*(Pp12+Pp21); Pp12 = s12; Pp21 = s12;

    /* -------------------------------------------------------------------
     * Measurement update — only when a fresh barometer sample is available.
     * H = [1, 0, 0]  →  S = Pp[0][0] + R,  K = Pp[:,0] / S
     * ------------------------------------------------------------------- */
    if (baro_new) {
        float innov = alt_baro_m - xp0;        /* innovation y = h_baro - h_pred */
        float S     = Pp00 + KF_R_H;            /* H*Pp*H' + R                    */

        if ((innov * innov / S) <= KF_GATE) {   /* chi-squared innovation gate     */
            float K0 = Pp00 / S;
            float K1 = Pp10 / S;
            float K2 = Pp20 / S;

            /* State update */
            xp0 += K0 * innov;
            xp1 += K1 * innov;
            xp2 += K2 * innov;

            /* Covariance update — Joseph form: P = (I-KH)*Pp*(I-KH)' + K*R*K'
             * I-KH = [[1-K0, 0, 0], [-K1, 1, 0], [-K2, 0, 1]]               */
            float a0 = 1.0f - K0;

            /* M = (I-KH)*Pp */
            float M00 = a0 *Pp00;  float M01 = a0 *Pp01;  float M02 = a0 *Pp02;
            float M10 = -K1*Pp00 + Pp10;
            float M11 = -K1*Pp01 + Pp11;
            float M12 = -K1*Pp02 + Pp12;
            float M20 = -K2*Pp00 + Pp20;
            float M21 = -K2*Pp01 + Pp21;
            float M22 = -K2*Pp02 + Pp22;

            /* P = M*(I-KH)' + K*R*K' */
            Pp00 = M00*a0  + K0*KF_R_H*K0;
            Pp01 = -K1*M00 + M01 + K0*KF_R_H*K1;
            Pp02 = -K2*M00 + M02 + K0*KF_R_H*K2;
            Pp10 = M10*a0  + K1*KF_R_H*K0;
            Pp11 = -K1*M10 + M11 + K1*KF_R_H*K1;
            Pp12 = -K2*M10 + M12 + K1*KF_R_H*K2;
            Pp20 = M20*a0  + K2*KF_R_H*K0;
            Pp21 = -K1*M20 + M21 + K2*KF_R_H*K1;
            Pp22 = -K2*M20 + M22 + K2*KF_R_H*K2;

            /* Symmetrise */
            s01 = 0.5f*(Pp01+Pp10); Pp01 = s01; Pp10 = s01;
            s02 = 0.5f*(Pp02+Pp20); Pp02 = s02; Pp20 = s02;
            s12 = 0.5f*(Pp12+Pp21); Pp12 = s12; Pp21 = s12;
        } else { dbg_kf_gate_reject++; }
    }

    /* -------------------------------------------------------------------
     * Store updated state and covariance
     * ------------------------------------------------------------------- */
    x[0] = xp0;
    x[1] = xp1;
    x[2] = xp2;

    /* A slowly growing negative bias is the usual cause of a steady vario drift.
       When the vertical acceleration is effectively zero and baro is stable, zero
       the bias and speed to prevent the filter from integrating a fake sink.
       Keep the clamp stricter than before so one bad update cannot bias the filter
       for many cycles. */
    if (fabsf(u) < 0.10f && fabsf(alt_baro_m - xp0) < 0.05f) {
        xp1 = 0.0f;
        xp2 = 0.0f;
        Pp11 = 0.01f;
        Pp22 = KF_P0_B;
    }

    if (x[2] >  1.0f) x[2] =  1.0f;
    if (x[2] < -1.0f) x[2] = -1.0f;

    P[0][0]=Pp00; P[0][1]=Pp01; P[0][2]=Pp02;
    P[1][0]=Pp10; P[1][1]=Pp11; P[1][2]=Pp12;
    P[2][0]=Pp20; P[2][1]=Pp21; P[2][2]=Pp22;

    *vz_out  = x[1];  /* m/s, up positive */
    *alt_out = x[0];  /* m,   above reference */
}
