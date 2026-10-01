/*
 * test_madgwick.c — Host-side characterization tests for the Madgwick AHRS filter.
 *
 * Author: Luca Obwegs
 *
 * Builds and runs natively (no ARM toolchain or hardware needed) because
 * madgwick.c has zero HAL/MCU dependencies. See tests/Makefile.
 */
#include "test_util.h"
#include "madgwick.h"
#include <math.h>

#define G 9.80665f

/* At rest, body-Z-down already aligned with gravity: the filter should report
 * az_world == az on the very first sample, with no convergence transient. */
static void test_rest_identity_orientation(void)
{
    Madgwick_Init();
    float az_world = Madgwick_Update(0.0f, 0.0f, -G, 0.0f, 0.0f, 0.0f);
    /* Tolerance set by fast_inv_sqrt's documented <0.2% error (see madgwick.c):
     * even at rest, that residual gets renormalised to a full-strength gradient
     * step, so a single update can be off by a bit more than the raw 0.2%. */
    CHECK_NEAR(az_world, -G, 0.05f, "az_world should equal -g at rest from identity orientation");
}

/* Madgwick_Init_FromAccel must solve for the quaternion that makes the
 * *current* accelerometer reading consistent with "at rest", regardless of
 * how the device is tilted at power-on. */
static void test_init_from_tilted_accel(void)
{
    /* Device tilted 30 deg from level; body reads partly on X, partly on Z. */
    const float deg30_rad = 0.5235988f;
    float ax = G * sinf(deg30_rad);
    float az = -G * cosf(deg30_rad);

    Madgwick_Init_FromAccel(ax, 0.0f, az);
    float az_world = Madgwick_Update(ax, 0.0f, az, 0.0f, 0.0f, 0.0f);

    CHECK_NEAR(az_world, -G, 0.05f, "az_world should be -g right after calibrating from a tilted reading");
}

/* With the gyro spinning the body (yaw has no gravity reference, so use roll)
 * and the accelerometer gate periodically re-anchoring attitude, az_world must
 * stay bounded by the physical accel magnitude. If the quaternion update ever
 * lost normalisation this would diverge far outside [-g, g]. */
static void test_bounded_during_rotation(void)
{
    Madgwick_Init();
    const float gyro_rate = 1.0f; /* rad/s roll rate */
    float max_abs_az_world = 0.0f;

    for (int i = 0; i < 2000; i++) {
        float az_world = Madgwick_Update(0.0f, 0.0f, -G, gyro_rate, 0.0f, 0.0f);
        CHECK(isfinite(az_world), "az_world must stay finite during sustained rotation");
        float a = fabsf(az_world);
        if (a > max_abs_az_world) max_abs_az_world = a;
    }
    CHECK(max_abs_az_world <= 1.05f * G, "az_world must stay within +-5% of g (quaternion normalisation holds)");
}

/* A large kinematic acceleration spike (e.g. a bump or a button press) must be
 * gated out of the attitude correction instead of corrupting the orientation. */
static void test_accel_gate_rejects_spike(void)
{
    Madgwick_Init();
    /* Prime with a few good samples first */
    for (int i = 0; i < 10; i++) {
        Madgwick_Update(0.0f, 0.0f, -G, 0.0f, 0.0f, 0.0f);
    }
    /* Huge spike: 3g, well outside MADGWICK_ACCEL_GATE_FRAC */
    float az_world = Madgwick_Update(0.0f, 0.0f, -3.0f * G, 0.0f, 0.0f, 0.0f);
    CHECK(isfinite(az_world), "az_world must stay finite when the accel gate rejects a spike");

    /* Attitude should recover cleanly once good samples resume */
    float recovered = 0.0f;
    for (int i = 0; i < 10; i++) {
        recovered = Madgwick_Update(0.0f, 0.0f, -G, 0.0f, 0.0f, 0.0f);
    }
    CHECK_NEAR(recovered, -G, 0.05f, "attitude should recover to -g after a rejected accel spike");
}

int main(void)
{
    test_rest_identity_orientation();
    test_init_from_tilted_accel();
    test_bounded_during_rotation();
    test_accel_gate_rejects_spike();
    TEST_SUMMARY();
}
