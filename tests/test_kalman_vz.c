/*
 * test_kalman_vz.c — Host-side characterization tests for the vertical-speed Kalman filter.
 *
 * Author: Luca Obwegs
 *
 * Builds and runs natively (no ARM toolchain or hardware needed). kalman_vz.c
 * only needs uint8_t and the tuning constants from filter_tuning.h; the one
 * MCU-side symbol it references (dbg_kf_gate_reject) is stubbed below instead
 * of linking the rest of the firmware. See tests/Makefile.
 */
#include "test_util.h"
#include "kalman_vz.h"
#include <math.h>

/* Normally defined in vario.c; stubbed here so this test links standalone. */
volatile uint32_t dbg_kf_gate_reject = 0;

#define G     9.80665f
#define KF_DT (1.0f / 26.0f) /* must match SAMPLE_HZ / KF_DT in kalman_vz.c */

/* At rest (az_world == -g, barometer flat at 0 m), velocity and altitude
 * must settle to ~0 and stay there - this is the filter's steady state
 * for the entire time the device is sitting still before launch. */
static void test_converges_to_rest(void)
{
    KalmanVz_Init(0.0f);
    float vz = 0.0f, alt = 0.0f;

    for (int i = 0; i < 400; i++) {
        uint8_t baro_new = (i % 3) == 0; /* ~8 Hz baro vs ~26 Hz IMU */
        KalmanVz_Update(-G, 0.0f, baro_new, &vz, &alt);
    }
    CHECK_NEAR(vz, 0.0f, 0.05f, "vertical speed should settle near 0 at rest");
    CHECK_NEAR(alt, 0.0f, 0.3f, "altitude should settle near 0 at rest");
}

/* With no barometer correction, a constant 1 m/s^2 upward specific force must
 * integrate to the textbook kinematic result: v = u*t, h = 0.5*u*t^2. This
 * pins down the sign conventions in F/B and the az_world -> a_up conversion. */
static void test_constant_acceleration_integrates_correctly(void)
{
    KalmanVz_Init(0.0f);
    const float u = 1.0f;          /* commanded a_up [m/s^2] */
    const float az_world = -(G + u);
    const int   n = 52;            /* 52 steps @ 1/26 s = 2.0 s */
    float vz = 0.0f, alt = 0.0f;

    for (int i = 0; i < n; i++) {
        KalmanVz_Update(az_world, 0.0f, 0, &vz, &alt);
    }
    float t = n * KF_DT;
    CHECK_NEAR(vz, u * t, 0.02f, "velocity should match v = u*t under pure IMU integration");
    CHECK_NEAR(alt, 0.5f * u * t * t, 0.05f, "altitude should match h = 0.5*u*t^2 under pure IMU integration");
}

/* KalmanVz_ZeroVelocity() is the ZUPT reset called when vario.c detects the
 * device is stationary; the very next update must not reintroduce the speed
 * that was just zeroed out. */
static void test_zupt_resets_velocity(void)
{
    KalmanVz_Init(0.0f);
    float vz = 0.0f, alt = 0.0f;

    /* Build up some velocity first */
    for (int i = 0; i < 52; i++) {
        KalmanVz_Update(-(G + 1.0f), 0.0f, 0, &vz, &alt);
    }
    CHECK(fabsf(vz) > 0.5f, "precondition: velocity should be non-trivial before ZUPT");

    KalmanVz_ZeroVelocity();
    KalmanVz_Update(-G, alt, 0, &vz, &alt); /* at rest, no further acceleration */
    CHECK_NEAR(vz, 0.0f, 0.01f, "velocity must be ~0 immediately after a ZUPT reset");
}

/* A single wildly-wrong barometer sample (e.g. a pressure-sensor glitch) must
 * be rejected by the chi-squared innovation gate instead of corrupting the
 * altitude estimate. */
static void test_gate_rejects_baro_outlier(void)
{
    KalmanVz_Init(0.0f);
    float vz = 0.0f, alt = 0.0f;

    /* Settle at rest first */
    for (int i = 0; i < 200; i++) {
        KalmanVz_Update(-G, 0.0f, (i % 3) == 0, &vz, &alt);
    }

    uint32_t rejects_before = dbg_kf_gate_reject;
    KalmanVz_Update(-G, 1000.0f, 1, &vz, &alt); /* obviously-glitched baro sample */

    CHECK(dbg_kf_gate_reject > rejects_before, "a 1000 m baro outlier must be rejected by the innovation gate");
    CHECK(fabsf(alt) < 1.0f, "altitude must not jump toward the rejected outlier");
}

int main(void)
{
    test_converges_to_rest();
    test_constant_acceleration_integrates_correctly();
    test_zupt_resets_velocity();
    test_gate_rejects_baro_outlier();
    TEST_SUMMARY();
}
