/*
 * filter_tuning.h  —  All user-accessible tuning parameters for the
 *                     Madgwick + Kalman state estimator and vario audio engine.
 *
 * Author: Luca Obwegs
 *
 * Effect notation used in every comment:
 *   ↑ increase value  →  [resulting change in behaviour]
 *   ↓ decrease value  →  [resulting change in behaviour]
 */
#ifndef INC_FILTER_TUNING_H_
#define INC_FILTER_TUNING_H_

/* ======================================================================
 * 1.  MADGWICK AHRS — attitude filter
 *
 *     Converts IMU (accel + gyro) into az_world, the world-frame vertical
 *     specific force that drives the Kalman velocity prediction.
 * ====================================================================== */

/* Gradient-descent step [rad/s].  Controls how fast attitude corrects toward
 * the accelerometer's gravity reference.
 * ↑ faster convergence from a wrong initial orientation; stronger gyro-bias rejection
 * ↓ attitude is less disturbed by short dynamic accelerations (banking, turbulence)
 * Practical range: 0.01 – 0.10.  Start here; reduce if velocity shows jitter in turns. */
#define MADGWICK_BETA            0.03f

/* Gyro-bias drift rate [rad/s²]  (must stay << MADGWICK_BETA).
 * Slowly estimates and cancels the IMU gyro's DC offset.
 * ↑ tracks slow thermal/temperature gyro drift faster
 * ↓ less likely to mistake a genuine rotation for gyro bias
 * Note: yaw (Z) is always locked to 0 — no magnetometer available. */
#define MADGWICK_ZETA            0.00005f

/* Accel magnitude gate: gradient correction is skipped when |a| deviates
 * from g by more than (GATE_FRAC × g).  Protects attitude estimation during
 * button presses, turbulence, pull-ups, or any kinematic acceleration spike.
 * ↑ gate is wider — attitude corrects even during moderate dynamic manoeuvres
 * ↓ gate is stricter — attitude is more stable but slower to recover after abrupt motion
 * At 0.25 the gate triggers above 1.25 g or below 0.75 g. */
#define MADGWICK_ACCEL_GATE_FRAC 0.25f


/* ======================================================================
 * 2.  KALMAN FILTER — process noise  (trust in the IMU-based prediction)
 *
 *     State: x = [altitude h (m),  vertical speed vz (m/s),  accel bias (m/s²)]
 *     Larger Q  →  filter believes states can change quickly  →  more responsive.
 *     Smaller Q →  filter believes states are smooth          →  more filtered.
 * ====================================================================== */

/* IMU acceleration process noise spectral density [m²/s³].
 * Governs how quickly the filter allows vz to change based on the IMU alone.
 * ↑ velocity responds faster to acceleration changes; estimate is noisier at rest
 * ↓ smoother velocity trace; slower to detect the onset of a thermal or sink
 * Rule of thumb: ≈ (expected peak accel uncertainty in m/s²)² / (sample rate) */
#define KF_Q_AZ                  0.06f
//#define KF_Q_AZ   7.79e-7f

/* Accelerometer-bias random-walk rate [m²/s⁵].
 * The az_bias state absorbs the systematic error in az_world (Madgwick attitude
 * drift, sensor temperature offset, etc.) to prevent velocity from drifting.
 * ↑ bias adapts faster → less velocity sag during steady climb; more reactive to transients
 * ↓ bias changes slowly → stable in cruise flight; slow to recover after aggressive manoeuvres */
#define KF_Q_BIAS                0.006f


/* ======================================================================
 * 3.  KALMAN FILTER — measurement noise  (trust in the barometer)
 * ====================================================================== */

/* Barometer altitude measurement variance [m²].
 * Derived as (sigma_p / (rho0 × g))² × 10 with sigma_p = 0.5 Pa (SPL06 @ 8× OS).
 * ↑ filter trusts baro less → velocity relies more on IMU; smoother but drifts more
 * ↓ filter trusts baro more → altitude and velocity closely follow the barometer;
 *   may cause jitter if baro is noisy (pressure turbulence, aircraft body effects) */
#define KF_R_H                   0.020f

/* Chi-squared innovation gate threshold (1 DOF, 99.9 % confidence).
 * Baro updates whose (innovation² / S) exceeds this value are rejected as outliers.
 * ↑ accepts larger baro spikes → less protection against sensor glitches
 * ↓ rejects more baro updates → safer; velocity corrects more slowly after large altitude jumps */
#define KF_GATE                  10.83f


/* ======================================================================
 * 4.  KALMAN FILTER — initial / post-ZUPT covariance
 * ====================================================================== */

/* Altitude uncertainty at power-on [m²]  (1-σ = √KF_P0_H ≈ 5 m). */
#define KF_P0_H                  25.0f

/* Velocity uncertainty at power-on and after every ZUPT reset [(m/s)²].
 * ↑ baro can correct velocity more aggressively right after ZUPT
 * ↓ velocity is trusted more precisely after ZUPT; less baro noise enters vz */
#define KF_P0_VZ                  4.0f

/* Accel-bias uncertainty at power-on [(m/s²)²].  Not reset on ZUPT — the
 * filter retains the learned bias value across stationary periods.
 * ↑ bias adapts faster from zero at startup
 * ↓ startup bias estimate changes slowly; safe for well-calibrated sensors */
#define KF_P0_B                  0.0024042f

/* Altitude–velocity cross-covariance seed injected after every ZUPT [m·m/s].
 * K1 = P[1][0] / (P[0][0] + R).  Without this seed K1 ≈ 0 and the barometer
 * cannot damp velocity overshoot for several seconds after a ZUPT event.
 * ↑ baro damps overshoot more aggressively → quicker settling; slightly more baro noise in vz
 * ↓ gentler damping; velocity is quieter right after standstill but overshoot lingers longer */
#define KF_P0_ZUPT_CROSS          0.004f


/* ======================================================================
 * 5.  ZERO-VELOCITY UPDATE (ZUPT)
 *
 *     Resets vz to 0 when barometer + velocity agree the device is stationary.
 *     Prevents slow IMU/Madgwick drift from accumulating into a false vario reading.
 * ====================================================================== */

/* Maximum baro altitude swing [cm] over the detection window to declare stationary.
 * Tightened so the filter snaps to zero more aggressively when the device is truly still.
 * ↑ more aggressive holding at rest; may false-zero in very gentle lift
 * ↓ more tolerant of tiny pressure noise; less aggressive at rest */
#define ZUPT_ALT_WINDOW_CM        2

/* IMU sample count for the ZUPT detection window (≈ 1.23 s at 26 Hz).
 * A slightly longer window makes the stationary check more robust before zeroing.
 * ↑ more reliable stationary detection; slower to clear after landing
 * ↓ faster reset; more likely to false-zero in short pauses */
#define ZUPT_SAMPLE_COUNT        32

/* Velocity gate for ZUPT [m/s].  ZUPT is suppressed when |vz_est| exceeds this.
 * Lower value = stronger zero-velocity hold while still on the bench.
 * ↑ more aggressive rest locking; may clip a very gentle thermal
 * ↓ less aggressive hold; more drift risk while static */
#define ZUPT_VZ_GATE_MS           0.02f
#define ZUPT_ACCEL_GATE_G         0.05f
#define ZUPT_ALT_GATE_CM          2

/* After a valid stationary event, hold zero-velocity for a short burst of samples
 * to prevent the filter from immediately drifting back negative on a tiny sensor offset.
 * At 26 Hz, 12 samples ≈ 462 ms. */
#define ZUPT_HOLD_SAMPLES         12


/* ======================================================================
 * 6.  STARTUP GRAVITY CALIBRATION
 * ====================================================================== */

/* Number of quiet IMU samples (|a| within ±10 % of g) averaged to estimate the
 * sensor's true gravity vector and mounting orientation.  The window extends
 * up to 10 s waiting for still samples; the device can be moved immediately
 * after power-on without corrupting the calibration.
 * ↑ more accurate g_eff and initial Madgwick quaternion; potentially longer startup
 * ↓ faster startup; noisier initial estimates */
#define GRAVITY_CAL_SAMPLES      64


/* ======================================================================
 * 7.  VARIO AUDIO ENGINE
 * ====================================================================== */

/* Minimum vertical speed [cm/s] above which the intermittent lift beep starts.
 * ↑ beeping starts only in stronger lift → quieter in weak conditions
 * ↓ even marginal lift triggers beeping → more sensitive but potentially distracting */
#define LIFT_THRESHOLD            10

/* Sink rate [cm/s, negative] at which the continuous sink alarm activates.
 * ↑ (less negative, e.g. −150) alarm starts sooner → more alert to sink
 * ↓ (more negative, e.g. −400) alarm only sounds in fast descent → less intrusive */
#define SINK_THRESHOLD          -250

/* Frequency [Hz] of the continuous sink tone. */
#define SINK_FREQUENCY           300

/* Beep cycle period [ms] at exactly LIFT_THRESHOLD — the slowest (laziest) beep.
 * ↑ slower rhythm at low climb rates → more relaxed audio
 * ↓ faster rhythm even at marginal lift → more urgent response */
#define BEEP_CYCLE_MAX_MS        400

/* Minimum beep cycle period [ms] — cap at high climb rates (physical buzzer limit).
 * ↑ beeping never becomes faster than this, even in strong thermals
 * ↓ very rapid beeping in strong lift → more expressive but may become a single tone */
#define BEEP_CYCLE_MIN_MS         80

/* Cycle-period reduction per additional cm/s of climb rate [ms / (cm/s)].
 * Total cycle = BEEP_CYCLE_MAX_MS − v_cm_s × BEEP_CYCLE_SLOPE_MS  (clamped to MIN).
 * ↑ beep rate ramps up faster with climb rate → more dynamic audio response
 * ↓ beep rate changes slowly → subtle variation; useful in gusty conditions */
#define BEEP_CYCLE_SLOPE_MS        5

#endif /* INC_FILTER_TUNING_H_ */
