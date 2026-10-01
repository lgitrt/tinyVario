/*
 * vario.c — Complementary filter, IMU/baro fusion, sensor lifecycle, HAL callbacks
 *
 * Author: Luca Obwegs
 */
#include "vario.h"
#include "buzzer.h"
#ifdef USE_MADGWICK_KALMAN
#include "madgwick.h"
#include "kalman_vz.h"
#include <math.h>
#endif

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim21;

// --- Public state ---
volatile uint8_t  imu_data_ready      = 0;
volatile uint32_t g_imu_dt_ticks      = 0;
uint8_t           sensors_initialized = 0;
uint32_t          last_imu_sample_tick = 0;
int32_t           current_altitude_cm  = 0;
int32_t           vertical_velocity_cm_s = 0;
int32_t           reference_pressure_pa  = 0;
LSM6DS3_Data_t    imu_data;
uint8_t           imu_rx_buffer[12];

// --- Private filter state ---
static uint8_t  filter_initialized  = 0;
static uint16_t last_capture_tick = 0;
static uint8_t  imu_first_capture = 1;

/* Variables only used by the original complementary filter */
#ifndef USE_MADGWICK_KALMAN
static int32_t  est_altitude_scaled = 0;
static int32_t  est_velocity_scaled = 0;
static int32_t  zupt_alt_min  = 0;
static int32_t  zupt_alt_max  = 0;
static uint16_t zupt_counter  = 0;
static uint8_t  zupt_window_init = 0;
static int32_t  resting_g_mg  = 1000;
static int32_t  bias_accumulator  = 0;
static int32_t  a_z_remainder = 0;
static int32_t  v_remainder   = 0;
#endif

#ifndef USE_MADGWICK_KALMAN
static uint32_t Integer_Sqrt(uint32_t x) {
    uint32_t res = 0, bit = 1UL << 30;
    while (bit > x) bit >>= 2;
    while (bit) {
        if (x >= res + bit) { x -= res + bit; res = (res >> 1) + bit; }
        else                 { res >>= 1; }
        bit >>= 2;
    }
    return res;
}
#endif

// --- HAL callbacks (override weak definitions) ---

// --- Debug counters ---
volatile uint32_t dbg_exti_count     = 0;
volatile uint32_t dbg_exti_i2c_busy  = 0;
volatile uint32_t dbg_dma_fail       = 0;
volatile uint32_t dbg_dma_complete   = 0;
volatile uint32_t dbg_process_calls  = 0;
volatile uint32_t dbg_baro_ready     = 0;
volatile uint32_t dbg_recover_imu    = 0;
volatile uint32_t dbg_zupt_count     = 0;
volatile uint32_t dbg_kf_gate_reject = 0;
volatile uint32_t dbg_accel_reject   = 0;
volatile uint32_t dbg_dt_zero        = 0;
volatile uint32_t dbg_pm_last_ticks  = 0;
volatile uint32_t dbg_pm_max_ticks   = 0;
volatile float    dbg_az_world_mps2   = 0.0f;
volatile uint32_t dbg_standby_fail   = 0;
volatile uint8_t  dbg_mcu_state      = DBG_MCU_ACTIVE;
volatile uint8_t  dbg_spl06_active   = 0;
volatile uint8_t  dbg_imu_active     = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == acc_INT_Pin) {
        uint16_t now_tick = __HAL_TIM_GET_COUNTER(&htim21);
        if (!imu_first_capture) {
            g_imu_dt_ticks = (uint16_t)(now_tick - last_capture_tick);
        } else {
            imu_first_capture = 0;
        }
        last_capture_tick = now_tick;
        dbg_exti_count++;
        if (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY) dbg_exti_i2c_busy++;
        if (LSM6DS3_ReadRaw_DMA(&hi2c1, imu_rx_buffer) != HAL_OK) dbg_dma_fail++;
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        dbg_dma_complete++;
        imu_data_ready = 1;
    }
}

// --- Sensor management ---

void Recover_IMU_Bus(void) {
    dbg_recover_imu++;
    HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
    HAL_I2C_DeInit(&hi2c1);
    HAL_Delay(2);
    if (HAL_I2C_Init(&hi2c1) == HAL_OK) {
        HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
        HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
    }
    LSM6DS3_Init(&hi2c1);
    dbg_imu_active = 1;
    uint8_t dummy[12];
    LSM6DS3_ReadRaw_Poll(&hi2c1, dummy);
    imu_first_capture  = 1;
    last_imu_sample_tick = HAL_GetTick();
    HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);
}

uint8_t Sensors_Safe_Shutdown(void) {
    if (!sensors_initialized) return 1;

    uint8_t baro_ok = 0, imu_ok = 0;

    for (uint8_t attempt = 0; attempt < SENSOR_SHUTDOWN_MAX_ATTEMPTS; attempt++) {
        if (!baro_ok) {
            SPL06_EnterStandby(&hi2c1);
            baro_ok = SPL06_IsInStandby(&hi2c1);
        }
        if (!imu_ok) {
            LSM6DS3_PowerDown(&hi2c1);
            imu_ok = LSM6DS3_IsPoweredDown(&hi2c1);
        }
        if (baro_ok && imu_ok) break;

        HAL_I2C_DeInit(&hi2c1);
        HAL_Delay(2);
        if (HAL_I2C_Init(&hi2c1) == HAL_OK) {
            HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE);
            HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0);
        }
        HAL_Delay(2);
    }

    dbg_spl06_active = baro_ok ? 0 : 1;  /* update reflects what I2C confirmed */
    dbg_imu_active   = imu_ok  ? 0 : 1;
    return baro_ok && imu_ok;
}

// --- Main filter ---

void Process_Vario_Math(void) {
    /* Protect imu_rx_buffer from a DMA overwrite while we parse it.
       EXTI stays masked through the SPL06 I2C reads; re-enabled before float computation. */
    HAL_NVIC_DisableIRQ(EXTI0_1_IRQn);
    LSM6DS3_ProcessData(imu_rx_buffer, &imu_data);
    dbg_process_calls++;

#ifdef USE_MADGWICK_KALMAN
    /* ----------------------------------------------------------------
     * Madgwick AHRS + Kalman vertical-speed filter
     * IMU data: accel in mg (fixed-point), gyro in mdps (fixed-point)
     * Both filters run at IMU rate (52 Hz); baro update gated by baro_new.
     * ---------------------------------------------------------------- */
    {
        static int32_t  p_cached_mk   = 0;
        static uint16_t cal_count_mk  = 0;  /* quiet samples only */
        static uint16_t cal_total_mk  = 0;  /* all samples — for 10 s timeout */
        static int32_t  cal_ax_sum    = 0;
        static int32_t  cal_ay_sum    = 0;
        static int32_t  cal_az_sum    = 0;
        /* ZUPT state */
        static int32_t  zupt_min_mk  = 0;
        static int32_t  zupt_max_mk  = 0;
        static uint16_t zupt_cnt_mk  = 0;
        static uint16_t zupt_hold_mk = 0;
        static uint8_t  zupt_rdy_mk  = 0;
        uint8_t baro_new = 0;
        uint16_t dbg_t0 = __HAL_TIM_GET_COUNTER(&htim21);
        static uint32_t last_pm_ms = 0;
        uint32_t now_ms = HAL_GetTick();
        uint8_t long_gap = (now_ms - last_pm_ms) > 200u;
        last_pm_ms = now_ms;

        /* EXTI already disabled; do SPL06 reads then re-enable for float computation */
        if (SPL06_DataReady(&hi2c1, 0x10)) {
            dbg_baro_ready++;
            SPL06_GetTemperature(&hi2c1);
            p_cached_mk = SPL06_GetPressure(&hi2c1);
            baro_new = 1;
        }
        HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

        /* Calibration: skip samples where |a| deviates from 1 g — device is being moved.
           Accumulates only quiet samples; 10-second timeout forces completion regardless. */
        if (!filter_initialized) {
            int32_t ax_ = imu_data.accel_mg[0];
            int32_t ay_ = imu_data.accel_mg[1];
            int32_t az_ = imu_data.accel_mg[2];
            int32_t mag_sq = ax_*ax_ + ay_*ay_ + az_*az_;  /* mg^2, fits in int32 */
            ++cal_total_mk;
            if (mag_sq > 810000L && mag_sq < 1210000L) {   /* 900-1100 mg = ±10% of g */
                cal_ax_sum += ax_; cal_ay_sum += ay_; cal_az_sum += az_;
                ++cal_count_mk;
            }
            uint8_t done = (cal_count_mk >= GRAVITY_CAL_SAMPLES) ||
                           (cal_total_mk  >= 10u * SAMPLE_HZ);
            if (done && p_cached_mk != 0) {
                reference_pressure_pa = p_cached_mk;
                if (cal_count_mk >= 8) {
                    float gc   = 9.80665e-3f / cal_count_mk;
                    float ax_m = cal_ax_sum * gc, ay_m = cal_ay_sum * gc, az_m = cal_az_sum * gc;
                    Madgwick_Init_FromAccel(ax_m, ay_m, az_m);
                    float g_eff = sqrtf(ax_m*ax_m + ay_m*ay_m + az_m*az_m);
                    KalmanVz_SetGravity(g_eff);
                } else {
                    /* Not enough still samples — use identity and nominal g; Madgwick converges in ~2 s */
                    Madgwick_Init();
                    KalmanVz_SetGravity(9.80665f);
                }
                KalmanVz_Init(0.0f);
                zupt_rdy_mk = 0;
                filter_initialized = 1;
            }
            return;
        }

        /* mg → m/s^2 */
        float ax = imu_data.accel_mg[0] * 9.80665e-3f;
        float ay = imu_data.accel_mg[1] * 9.80665e-3f;
        float az = imu_data.accel_mg[2] * 9.80665e-3f;
        /* mdps → rad/s */
        float gx = imu_data.gyro_dps[0] * 1.74533e-5f;
        float gy = imu_data.gyro_dps[1] * 1.74533e-5f;
        float gz = imu_data.gyro_dps[2] * 1.74533e-5f;

        /* Baro altitude: ISA sea-level scale 1/(rho0*g) = 0.08325 m/Pa, accurate to <10% up to 3000m */
        float   alt_baro_m = (float)(reference_pressure_pa - p_cached_mk) * 0.08325f;
        int32_t baro_cm_mk = (int32_t)(alt_baro_m * 100.0f);

        /* Madgwick attitude → world-frame vertical specific force [m/s^2] */
        float az_world = Madgwick_Update(ax, ay, az, gx, gy, gz);
        dbg_az_world_mps2 = az_world;

        /* Kalman filter; suppress baro update if resuming after a long gap to avoid stale-state mismatch */
        float vz_ms = 0.0f, alt_m = 0.0f;
        KalmanVz_Update(az_world, alt_baro_m, (baro_new && !long_gap), &vz_ms, &alt_m);

        /* ZUPT: only force vz=0 when the device is truly still. A constant-speed
           climb/sink is not stationary even if acceleration is briefly small, so the
           filter must still track the barometer-driven motion instead of zeroing it. */
        if (zupt_hold_mk > 0) {
            zupt_hold_mk--;
            dbg_zupt_count++;
            KalmanVz_ZeroVelocity();
            vz_ms = 0.0f;
        } else {
            if (!zupt_rdy_mk) {
                zupt_min_mk = zupt_max_mk = baro_cm_mk;
                zupt_cnt_mk = 0;
                zupt_rdy_mk = 1;
            } else {
                if (baro_cm_mk < zupt_min_mk) zupt_min_mk = baro_cm_mk;
                if (baro_cm_mk > zupt_max_mk) zupt_max_mk = baro_cm_mk;
                zupt_cnt_mk++;
            }
            if (zupt_cnt_mk >= ZUPT_SAMPLE_COUNT) {
                float accel_mag = sqrtf(ax*ax + ay*ay + az*az);
                float grav_error_g = fabsf(accel_mag - 9.80665f) / 9.80665f;
                uint8_t stationary = ((zupt_max_mk - zupt_min_mk) <= ZUPT_ALT_GATE_CM) &&
                                     (fabsf(vz_ms) < ZUPT_VZ_GATE_MS) &&
                                     (grav_error_g < ZUPT_ACCEL_GATE_G);
                if (stationary) {
                    dbg_zupt_count++;
                    KalmanVz_ZeroVelocity();
                    vz_ms = 0.0f;
                    zupt_hold_mk = ZUPT_HOLD_SAMPLES;
                }
                zupt_min_mk = zupt_max_mk = baro_cm_mk;
                zupt_cnt_mk = 0;
            }
        }

        current_altitude_cm    = (int32_t)(alt_m * 100.0f);
        vertical_velocity_cm_s = (int32_t)(vz_ms * 100.0f);
        /* Record execution time; wraps correctly with uint16_t arithmetic at 524 kHz */
        uint16_t dbg_elapsed = (uint16_t)(__HAL_TIM_GET_COUNTER(&htim21) - dbg_t0);
        dbg_pm_last_ticks = dbg_elapsed;
        if (dbg_elapsed > dbg_pm_max_ticks) dbg_pm_max_ticks = dbg_elapsed;
        Update_Buzzer(vertical_velocity_cm_s);
    }

#else
    /* ----------------------------------------------------------------
     * Original integer complementary filter (unchanged)
     * ---------------------------------------------------------------- */
    {
        static int32_t  cal_sum   = 0;
        static uint16_t cal_count = 0;
        static int32_t  p_cached  = 0;
        static uint16_t zupt_hold = 0;

        if (SPL06_DataReady(&hi2c1, 0x10)) {   // PRS_RDY only
            dbg_baro_ready++;
            SPL06_GetTemperature(&hi2c1);
            p_cached = SPL06_GetPressure(&hi2c1);
        }
        int32_t p = p_cached;

        // --- Gravity calibration pass ---
        if (!filter_initialized) {
            int32_t ax = imu_data.accel_mg[0];
            int32_t ay = imu_data.accel_mg[1];
            int32_t az = imu_data.accel_mg[2];
            cal_sum += (int32_t)Integer_Sqrt((uint32_t)((ax*ax) + (ay*ay) + (az*az)));
            if (++cal_count >= GRAVITY_CAL_SAMPLES) {
                resting_g_mg          = cal_sum / GRAVITY_CAL_SAMPLES;
                reference_pressure_pa = p;
                est_altitude_scaled   = 0;
                est_velocity_scaled   = 0;
                bias_accumulator      = 0;
                a_z_remainder         = 0;
                v_remainder           = 0;
                filter_initialized    = 1;
                zupt_window_init      = 0;
            }
            return;
        }

        // --- Baro altitude ---
        int32_t baro_altitude_cm = ((reference_pressure_pa - p) * 850) / 100;
        int32_t baro_alt_scaled  = baro_altitude_cm << 10;

        // --- Vertical acceleration ---
        int32_t ax = imu_data.accel_mg[0];
        int32_t ay = imu_data.accel_mg[1];
        int32_t az = imu_data.accel_mg[2];
        int32_t accel_mag_mg = (int32_t)Integer_Sqrt((uint32_t)((ax*ax) + (ay*ay) + (az*az)));

        if (accel_mag_mg > 3000 || accel_mag_mg < 200) { dbg_accel_reject++; return; }

        int32_t vertical_accel_cms2 = ((accel_mag_mg - resting_g_mg) * 981) / 1000;
        int32_t a_z_scaled = (vertical_accel_cms2 << 10) - (bias_accumulator / DIV_BIAS);

        // --- Timer clock for lossless integration ---
        uint32_t timer_clk = HAL_RCC_GetPCLK2Freq();
        if ((RCC->CFGR & RCC_CFGR_PPRE2) != RCC_CFGR_PPRE2_DIV1) timer_clk *= 2;
        timer_clk /= (TIM21_PRESCALER + 1);

        uint32_t dt_ticks = g_imu_dt_ticks;
        if (dt_ticks == 0) { dbg_dt_zero++; return; }

        // --- Prediction: accel → velocity ---
        int64_t az_num     = (int64_t)a_z_scaled * dt_ticks + a_z_remainder;
        int64_t delta_v_64 = az_num / timer_clk;
        a_z_remainder = (int32_t)(az_num - delta_v_64 * timer_clk);
        int32_t pred_velocity_scaled = est_velocity_scaled + (int32_t)delta_v_64;

        // --- Prediction: velocity → altitude ---
        int64_t v_num        = (int64_t)est_velocity_scaled * dt_ticks + v_remainder;
        int64_t delta_alt_64 = v_num / timer_clk;
        v_remainder = (int32_t)(v_num - delta_alt_64 * timer_clk);
        int32_t pred_altitude_scaled = est_altitude_scaled + (int32_t)delta_alt_64;

        // --- Correction ---
        int32_t altitude_error_scaled = baro_alt_scaled - pred_altitude_scaled;
        est_altitude_scaled = pred_altitude_scaled + (altitude_error_scaled / DIV_ALTITUDE);
        est_velocity_scaled = pred_velocity_scaled + (altitude_error_scaled / DIV_VELOCITY);

        // Leaky bias integrator
        bias_accumulator -= altitude_error_scaled;
        bias_accumulator -= bias_accumulator / BIAS_LEAK_DIV;
        if (bias_accumulator >  BIAS_MAX_SCALED) bias_accumulator =  BIAS_MAX_SCALED;
        if (bias_accumulator < -BIAS_MAX_SCALED) bias_accumulator = -BIAS_MAX_SCALED;

        // --- ZUPT: zero velocity aggressively when the baro shows the device is still ---
        if (zupt_hold > 0) {
            zupt_hold--;
            dbg_zupt_count++;
            est_velocity_scaled = 0;
            v_remainder = 0;
        } else {
            if (!zupt_window_init) {
                zupt_alt_min = zupt_alt_max = baro_altitude_cm;
                zupt_window_init = 1;
                zupt_counter = 0;
            } else {
                if (baro_altitude_cm < zupt_alt_min) zupt_alt_min = baro_altitude_cm;
                if (baro_altitude_cm > zupt_alt_max) zupt_alt_max = baro_altitude_cm;
                zupt_counter++;
            }
            if (zupt_counter >= ZUPT_SAMPLE_COUNT) {
                if ((zupt_alt_max - zupt_alt_min) <= ZUPT_ALT_WINDOW_CM) {
                    dbg_zupt_count++;
                    est_velocity_scaled = 0;
                    v_remainder = 0;
                    zupt_hold = ZUPT_HOLD_SAMPLES;
                }
                zupt_alt_min = zupt_alt_max = baro_altitude_cm;
                zupt_counter = 0;
            }
        }

        current_altitude_cm    = est_altitude_scaled >> 10;
        vertical_velocity_cm_s = est_velocity_scaled >> 10;
        Update_Buzzer(vertical_velocity_cm_s);
    }
#endif
}
