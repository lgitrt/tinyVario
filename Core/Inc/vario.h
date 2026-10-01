/*
 * vario.h — Complementary filter, IMU/baro fusion, sensor lifecycle
 *
 * Author: Luca Obwegs
 */
#ifndef INC_VARIO_H_
#define INC_VARIO_H_

#include "main.h"
#include "lsm6ds3.h"
#include "spl06.h"
#include "filter_tuning.h"  /* ZUPT_*, GRAVITY_CAL_SAMPLES and all filter/audio params */

// --- Filter selection ---
// Uncomment to use Madgwick attitude + Kalman vertical-speed filter instead
// of the original integer complementary filter.
#define USE_MADGWICK_KALMAN

// --- IMU / ODR ---
#define SAMPLE_HZ                26   // LSM6DS3 ODR (CTRL1_XL/CTRL2_G = 0x20)
#define TIM21_PRESCALER           3   // must match htim21.Init.Prescaler exactly
#define IMU_STALL_TIMEOUT_MS    500   // max gap before bus-recovery fires (~19 ms nominal)
#define SENSOR_SHUTDOWN_MAX_ATTEMPTS 3

// --- Complementary filter tuning ---
// Lower divisor = stronger correction from barometer
#define DIV_ALTITUDE      8    // altitude pull toward baro
#define DIV_VELOCITY     16    // velocity pull toward baro
#define DIV_BIAS        512    // bias integrator correction speed
#define BIAS_LEAK_DIV  4096    // bias accumulator bleed-off per sample
#define BIAS_MAX_CORRECTION_CMS2  50
#define BIAS_MAX_SCALED  (BIAS_MAX_CORRECTION_CMS2 * 1024 * DIV_BIAS)

// --- Zero-velocity update (ZUPT) — thresholds now in filter_tuning.h ---

// Public state (written here, read by main and debug)
extern volatile uint8_t  imu_data_ready;
extern volatile uint32_t g_imu_dt_ticks;
extern uint8_t           sensors_initialized;
extern uint32_t          last_imu_sample_tick;
extern int32_t           current_altitude_cm;
extern int32_t           vertical_velocity_cm_s;
extern int32_t           reference_pressure_pa;
extern LSM6DS3_Data_t    imu_data;
extern uint8_t           imu_rx_buffer[12];

void    Process_Vario_Math(void);
void    Recover_IMU_Bus(void);
uint8_t Sensors_Safe_Shutdown(void);

// --- Debug counters (watch these in the debugger; they only accumulate, never reset) ---
extern volatile uint32_t dbg_exti_count;      /* IMU EXTI fires; expected ~52/s */
extern volatile uint32_t dbg_exti_i2c_busy;   /* I2C not READY when EXTI fires — SPL06 conflict */
extern volatile uint32_t dbg_dma_fail;        /* LSM6DS3_ReadRaw_DMA returned non-OK */
extern volatile uint32_t dbg_dma_complete;    /* DMA RX done; should track dbg_exti_count */
extern volatile uint32_t dbg_process_calls;   /* Process_Vario_Math entries */
extern volatile uint32_t dbg_baro_ready;      /* SPL06 PRS_RDY=1; expected ~8/s */
extern volatile uint32_t dbg_recover_imu;     /* Recover_IMU_Bus — stall watchdog fired */
extern volatile uint32_t dbg_zupt_count;      /* ZUPT firings (both filter paths) */
extern volatile uint32_t dbg_kf_gate_reject;  /* KF baro innovation rejected by chi2 gate */
extern volatile uint32_t dbg_accel_reject;    /* original filter: accel magnitude out of [200,3000] mg */
extern volatile uint32_t dbg_dt_zero;         /* original filter: dt_ticks==0 early return */
/* TIM21 ticks at ~524 kHz (2.097 MHz / 4); 1 tick ≈ 1.9 µs. IMU period = ~10 000 ticks. */
extern volatile uint32_t dbg_pm_last_ticks;   /* Process_Vario_Math duration, last call */
extern volatile uint32_t dbg_pm_max_ticks;    /* Process_Vario_Math duration, worst case */
extern volatile float    dbg_az_world_mps2;     /* Madgwick vertical specific force, NED-down [m/s²] */
extern volatile uint32_t dbg_standby_fail;    /* increments if HAL_PWR_EnterSTANDBYMode() returns — should stay 0 */

/* --- System state visibility (read in debugger watch window) ------------ */
typedef enum {
    DBG_MCU_ACTIVE   = 0,  /* CPU executing; all clocks running                 */
    DBG_MCU_SLEEP    = 1,  /* WFI sleep; wakes on EXTI/SysTick, SWD still works */
    DBG_MCU_STANDBY  = 2   /* deep standby; RAM lost on wake, MCU resets          */
} DbgMcuState_t;
extern volatile uint8_t dbg_mcu_state;    /* current MCU power mode                     */
extern volatile uint8_t dbg_spl06_active; /* 1=SPL06 measuring background, 0=standby    */
extern volatile uint8_t dbg_imu_active;   /* 1=LSM6DS3 active ODR, 0=powered down       */

#endif /* INC_VARIO_H_ */
