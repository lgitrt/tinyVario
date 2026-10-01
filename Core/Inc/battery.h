/*
 * battery.h — Battery ADC reading and critical-voltage handling
 *
 * Author: Luca Obwegs
 */
#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

#include "main.h"

// --- Battery protection thresholds (mV) ---
#define BAT_SHUTDOWN_MV        3200   // hard cutoff — safe margin above cell damage floor
#define BATT_CHECK_INTERVAL_MS 2000   // periodic check period while flying
#define LOW_BATT_WARN_MS       3000   // low-battery LED flash duration before auto-shutdown

// --- ADC / divider / calibration ---
/* Hardware divider on the PCB: VBAT -> 1M -> BATVOLTAGE -> 2.2M -> GND.
   The ADC measures the middle node, so Vadc = Vbat * 2.2 / (1 + 2.2) = 0.6875 Vbat.
   We recover the battery voltage with the inverse ratio: Vbat = Vadc * (1 + 2.2) / 2.2. */
#define BATTERY_TOP_MOHM                1000u
#define BATTERY_BOTTOM_MOHM             2200u
#define ADC_RAW_MAX_COUNTS              4095u
#define BATTERY_FULL_MV                 4200u
#define BATTERY_CHARGE_RESET_MV         4200u
#define BATTERY_VREFINT_MV              3000.0f
#define BATTERY_NOMINAL_VDD_MV         3000u
#define BATTERY_VOLTAGE_CAL_OFFSET_MV   0     // Small board calibration offset; keep at zero unless bench verifies a bias.
#define BATTERY_CHARGING_SOC_CAP_PCT      95   // stay conservative while USB is attached
#define BATT_SOC_TEMP_COMP_MV_PER_C     8
#define BATTERY_SAMPLE_QUIESCENT_MS     25u
#define EEPROM_BATTERY_MAH_ADDR        0x08080000U
#define BATTERY_FULL_SOH_SOC_PCT          96u

// --- Coulomb counting / fusion tuning ---
#define BATTERY_CAPACITY_MAH            90.0f
#define BATTERY_STATE_OFF_CURRENT_MA     0.04f
#define BATTERY_STATE_ACTIVE_CURRENT_MA  24.0f
#define BATTERY_STATE_CHARGING_CURRENT_MA -90.0f
#define BATTERY_VOLTAGE_SOC_WEIGHT       0.70f
#define BATTERY_COULOMB_SOC_WEIGHT       0.30f

// --- SOC LUT entries (voltage in mV, SOC in %). Tune here later with measured data. ---
#define BATTERY_SOC_LUT_LEN 13u
typedef struct {
    uint16_t voltage_mv;
    uint8_t  soc_pct;
} BatterySocPoint_t;

static const BatterySocPoint_t battery_soc_lut[BATTERY_SOC_LUT_LEN] = {
    { 4200, 100 },
    { 4100,  97 },
    { 4050,  95 },
    { 4000,  90 },
    { 3900,  80 },
    { 3800,  68 },
    { 3700,  56 },
    { 3600,  44 },
    { 3500,  33 },
    { 3400,  23 },
    { 3300,  14 },
    { 3200,   5 },
    { 3000,   0 }
};

extern volatile uint16_t dbg_battery_raw;
extern volatile uint16_t dbg_battery_vrefint_raw;
extern volatile float    dbg_battery_vdda_v;
extern volatile uint32_t dbg_battery_mv;
extern volatile uint8_t  battery_soc_pct;

uint32_t Read_Battery_MV(void);
uint32_t Battery_Read_MV_Quiescent(void);
uint8_t  Battery_Load_Is_Quiescent(void);
uint8_t  Battery_Get_SOC_From_Voltage(uint32_t mv, int32_t temp_c);
uint8_t  Battery_Get_Fused_SOC(uint32_t mv, int32_t temp_c);
float    Battery_Get_MAh_Consumed(void);
void     Battery_Load_State_From_EEPROM(void);
void     Battery_Save_State_To_EEPROM(void);
void     Battery_Update_Estimate(uint32_t battery_mv_mV);
void     Battery_Reset_Fusion_Consumed(void);
void     Handle_Critical_Battery(void);

#endif /* INC_BATTERY_H_ */
