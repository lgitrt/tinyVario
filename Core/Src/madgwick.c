/*
 * madgwick.c — Madgwick AHRS filter, NED / body-Z-down convention
 *
 * Author: Luca Obwegs
 *
 * Direct C translation of fcn_madgwick.m.
 * Gravity reference: NED [0, 0, 1] (down positive).
 * At rest the accelerometer reads the reaction force [0, 0, -g],
 * so the objective function F encodes expected = -R_nb(:,3).
 *
 * NOTE: links sqrtf() from libm. If linker reports undefined reference,
 *       add -lm to Project → Properties → MCU GCC Linker → Libraries.
 */
#include "madgwick.h"
#include <math.h>
#include <stdint.h>

/* ~21x faster than sqrtf on Cortex-M0+ soft-fp; single Newton-Raphson step, error <0.2% */
static __attribute__((always_inline)) inline float fast_inv_sqrt(float x)
{
    union { float f; uint32_t u; } v = {x};
    v.u = 0x5F3759DFul - (v.u >> 1);
    return v.f * (1.5f - 0.5f * x * v.f * v.f);
}

static float q[4];   /* quaternion [w, x, y, z], body→NED */
static float gb[3];  /* estimated gyro bias [rad/s]; Z locked to 0 (no magnetometer) */

void Madgwick_Init(void)
{
    q[0] = 1.0f; q[1] = 0.0f; q[2] = 0.0f; q[3] = 0.0f;
    gb[0] = 0.0f; gb[1] = 0.0f; gb[2] = 0.0f;
}

void Madgwick_Init_FromAccel(float ax, float ay, float az)
{
    float n = sqrtf(ax*ax + ay*ay + az*az);
    if (n < 1e-6f) { Madgwick_Init(); return; }
    float axn = ax/n, ayn = ay/n, azn = az/n;

    /* Find the quaternion that rotates body gravity direction (-accel_norm)
     * onto the NED-down unit vector [0,0,1], with yaw = 0.
     * Uses the half-vector method; singularities handled explicitly.     */
    if (azn < -0.999f) {
        /* Body-Z already points down — identity */
        q[0]=1.0f; q[1]=0.0f; q[2]=0.0f; q[3]=0.0f;
    } else if (azn > 0.999f) {
        /* Body-Z points up — 180 deg rotation around X */
        q[0]=0.0f; q[1]=1.0f; q[2]=0.0f; q[3]=0.0f;
    } else {
        /* General tilt: q = normalize([1-azn, -ayn, axn, 0]) */
        float qw = 1.0f - azn;
        float qx = -ayn;
        float qy =  axn;
        float qn = sqrtf(qw*qw + qx*qx + qy*qy);
        q[0]=qw/qn; q[1]=qx/qn; q[2]=qy/qn; q[3]=0.0f;
    }
    gb[0] = 0.0f; gb[1] = 0.0f; gb[2] = 0.0f;
}

__attribute__((optimize("O3")))
float Madgwick_Update(float ax, float ay, float az,
                      float gx, float gy, float gz)
{
    float w = q[0], x = q[1], y = q[2], z = q[3];

    /* Normalise accelerometer using squared magnitude to avoid sqrtf in the gate check.
     * |a|^2 comparisons are equivalent to |a| comparisons and save one sqrtf call.   */
    float a_sq = ax*ax + ay*ay + az*az;
    static const float g_ref = 9.80665f;
    static const float g_low_sq  = (g_ref*(1.0f-MADGWICK_ACCEL_GATE_FRAC)) *
                                   (g_ref*(1.0f-MADGWICK_ACCEL_GATE_FRAC));
    static const float g_high_sq = (g_ref*(1.0f+MADGWICK_ACCEL_GATE_FRAC)) *
                                   (g_ref*(1.0f+MADGWICK_ACCEL_GATE_FRAC));
    if (a_sq < 1e-12f || a_sq < g_low_sq || a_sq > g_high_sq) {
        float gxc = gx - gb[0], gyc = gy - gb[1], gzc = gz;
        float nw = w + MADGWICK_DT * 0.5f * (-x*gxc - y*gyc - z*gzc);
        float nx = x + MADGWICK_DT * 0.5f * ( w*gxc + y*gzc - z*gyc);
        float ny = y + MADGWICK_DT * 0.5f * ( w*gyc - x*gzc + z*gxc);
        float nz = z + MADGWICK_DT * 0.5f * ( w*gzc + x*gyc - y*gxc);
        float n_sq = nw*nw + nx*nx + ny*ny + nz*nz;
        if (n_sq > 1e-12f) {
            float inv_n = fast_inv_sqrt(n_sq);
            q[0]=nw*inv_n; q[1]=nx*inv_n; q[2]=ny*inv_n; q[3]=nz*inv_n;
        } else { q[0]=1.0f; q[1]=0.0f; q[2]=0.0f; q[3]=0.0f; }
        /* Full rotation avoids az_world = R33*az (wrong when tilted); always gives -g at rest */
        float R31 = 2.0f*(q[1]*q[3] - q[0]*q[2]);
        float R32 = 2.0f*(q[2]*q[3] + q[0]*q[1]);
        float R33 = q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3];
        return R31*ax + R32*ay + R33*az;
    }

    float inv_a = fast_inv_sqrt(a_sq);
    float axn = ax * inv_a;
    float ayn = ay * inv_a;
    float azn = az * inv_a;

    /* Objective function F = expected_body_accel - measured_normalised
     * Expected: sensor reads reaction to gravity = -(R_nb * [0,0,1])
     *   R_nb col-3 = [2(xz-wy), 2(yz+wx), w^2-x^2-y^2+z^2]
     * So F = -R_nb_col3 - a_normalised                              */
    float F1 = -2.0f*(x*z - w*y) - axn;
    float F2 = -2.0f*(y*z + w*x) - ayn;
    float F3 = -(w*w - x*x - y*y + z*z) - azn;

    /* Normalised gradient  (J^T * F) */
    float gw  =  2.0f*y*F1 - 2.0f*x*F2 - 2.0f*w*F3;
    float gx_ = -2.0f*z*F1 - 2.0f*w*F2 + 2.0f*x*F3;
    float gy_ =  2.0f*w*F1 - 2.0f*z*F2 + 2.0f*y*F3;
    float gz_ = -2.0f*x*F1 - 2.0f*y*F2 - 2.0f*z*F3;

    float gn_sq = gw*gw + gx_*gx_ + gy_*gy_ + gz_*gz_;
    if (gn_sq > 1e-20f) {
        float inv_gn = fast_inv_sqrt(gn_sq);
        gw *= inv_gn; gx_ *= inv_gn; gy_ *= inv_gn; gz_ *= inv_gn;
    }

    /* Gyro-bias update — Z-axis locked: yaw unobservable without magnetometer */
    gb[0] += MADGWICK_ZETA * gx_ * MADGWICK_DT;
    gb[1] += MADGWICK_ZETA * gy_ * MADGWICK_DT;
    /* gb[2] = 0 always */

    float gxc = gx - gb[0];
    float gyc = gy - gb[1];
    float gzc = gz; /* gb[2] = 0 */

    /* Quaternion derivative + gradient correction, then integrate */
    float nw = w + MADGWICK_DT * (0.5f*(-x*gxc - y*gyc - z*gzc) - MADGWICK_BETA*gw);
    float nx = x + MADGWICK_DT * (0.5f*( w*gxc + y*gzc - z*gyc) - MADGWICK_BETA*gx_);
    float ny = y + MADGWICK_DT * (0.5f*( w*gyc - x*gzc + z*gxc) - MADGWICK_BETA*gy_);
    float nz = z + MADGWICK_DT * (0.5f*( w*gzc + x*gyc - y*gxc) - MADGWICK_BETA*gz_);

    float n_sq = nw*nw + nx*nx + ny*ny + nz*nz;
    if (n_sq > 1e-12f) {
        float inv_n = fast_inv_sqrt(n_sq);
        q[0]=nw*inv_n; q[1]=nx*inv_n; q[2]=ny*inv_n; q[3]=nz*inv_n;
    } else { q[0]=1.0f; q[1]=0.0f; q[2]=0.0f; q[3]=0.0f; }

    /* Rotate body specific force to NED, return Z (down) component = az_world */
    w = q[0]; x = q[1]; y = q[2]; z = q[3];
    float R31 = 2.0f*(x*z - w*y);
    float R32 = 2.0f*(y*z + w*x);
    float R33 = w*w - x*x - y*y + z*z;
    return R31*ax + R32*ay + R33*az; /* NED-down [m/s^2]; ≈ -g at rest */
}
