/*
 * battery.c — Battery ADC reading, SOC estimation, and critical-voltage handling
 *
 * Author: Luca Obwegs
 */
#include "battery.h"
#include "button.h"
#include "buzzer.h"
#include "led.h"
#include "spl06.h"
#include "vario.h"
#include "stm32l0xx.h"
#include "stm32l0xx_ll_adc.h"

extern ADC_HandleTypeDef hadc;
extern I2C_HandleTypeDef hi2c1;

static uint32_t battery_last_mv = 0;
static float mah_consumed_model = 0.0f;
static uint32_t last_mah_update_ms = 0;
static uint8_t battery_was_full_and_charging = 0;

volatile uint16_t dbg_battery_raw = 0;
volatile uint16_t dbg_battery_vrefint_raw = 0;
volatile float    dbg_battery_vdda_v = 0.0f;
volatile uint32_t dbg_battery_mv = 0;

static uint16_t Battery_Read_Adc_Raw(uint32_t channel) {
    ADC_ChannelConfTypeDef sConfig = {0};

    /* The ADC channel selection register must be reset before switching channels.
       Otherwise the previous external channel remains selected alongside VREFINT,
       and the VREFINT read can effectively track the battery pin. */
    HAL_ADC_Stop(&hadc);
    __HAL_ADC_CLEAR_FLAG(&hadc, ADC_FLAG_EOC | ADC_FLAG_OVR);
    hadc.Instance->CHSELR = 0U;
    ADC->CCR &= ~(ADC_CCR_VREFEN | ADC_CCR_TSEN);

    sConfig.Channel = channel;
    sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
    if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
        return 0;
    }

    HAL_ADC_Start(&hadc);
    if (HAL_ADC_PollForConversion(&hadc, 10) != HAL_OK) {
        HAL_ADC_Stop(&hadc);
        __HAL_ADC_CLEAR_FLAG(&hadc, ADC_FLAG_EOC | ADC_FLAG_OVR);
        return 0;
    }

    uint16_t raw = HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);
    __HAL_ADC_CLEAR_FLAG(&hadc, ADC_FLAG_EOC | ADC_FLAG_OVR);
    return raw;
}

static uint32_t Battery_Read_Adc_MV(uint32_t channel) {
    uint16_t raw = Battery_Read_Adc_Raw(channel);
    if (raw == 0) {
        return battery_last_mv;
    }

    uint16_t vrefint_raw = Battery_Read_Adc_Raw(ADC_CHANNEL_VREFINT);
    uint32_t vdda_mv = BATTERY_NOMINAL_VDD_MV;
    if (vrefint_raw != 0U && vrefint_raw < ADC_RAW_MAX_COUNTS) {
        uint16_t vrefint_cal = *(uint16_t *)VREFINT_CAL_ADDR;
        if (vrefint_cal != 0U) {
            vdda_mv = ((uint32_t)VREFINT_CAL_VREF * (uint32_t)vrefint_cal) / (uint32_t)vrefint_raw;
        }
    }

    float vdda = (float)vdda_mv / 1000.0f;
    float adc_v = (float)raw * vdda / (float)ADC_RAW_MAX_COUNTS;
    float battery_mv = adc_v * (float)(BATTERY_TOP_MOHM + BATTERY_BOTTOM_MOHM) / (float)BATTERY_BOTTOM_MOHM * 1000.0f;

    dbg_battery_raw = raw;
    dbg_battery_vrefint_raw = vrefint_raw;
    dbg_battery_vdda_v = vdda;

    battery_mv += (float)BATTERY_VOLTAGE_CAL_OFFSET_MV;
    if (battery_mv < 0.0f) {
        battery_mv = 0.0f;
    }

    uint32_t battery_mV = (uint32_t)(battery_mv + 0.5f);
    dbg_battery_mv = battery_mV;
    battery_last_mv = battery_mV;
    return battery_mV;
}

uint32_t Read_Battery_MV(void) {
    return Battery_Read_Adc_MV(ADC_CHANNEL_1);
}

uint8_t Battery_Load_Is_Quiescent(void) {
    if (current_state == STATE_OFF || current_state == STATE_CHARGING) {
        return 1;
    }

    if (current_state != STATE_ACTIVE) {
        return 1;
    }

    if (is_beeping) {
        return 0;
    }

    return ((HAL_GetTick() - last_imu_sample_tick) > BATTERY_SAMPLE_QUIESCENT_MS);
}

uint32_t Battery_Read_MV_Quiescent(void) {
    if (!Battery_Load_Is_Quiescent()) {
        return battery_last_mv;
    }
    return Read_Battery_MV();
}

static int32_t Battery_Temperature_Compensated_MV(uint32_t mv, int32_t temp_c) {
    int32_t temp_adj_mv = (25 - temp_c) * BATT_SOC_TEMP_COMP_MV_PER_C;
    return (int32_t)mv + temp_adj_mv;   // add back what cold takes away
}

uint8_t Battery_Get_SOC_From_Voltage(uint32_t mv, int32_t temp_c) {
    int32_t compensated_mv = Battery_Temperature_Compensated_MV(mv, temp_c);
    if (compensated_mv >= battery_soc_lut[0].voltage_mv) {
        return battery_soc_lut[0].soc_pct;
    }
    if (compensated_mv <= battery_soc_lut[(sizeof(battery_soc_lut) / sizeof(battery_soc_lut[0])) - 1].voltage_mv) {
        return battery_soc_lut[(sizeof(battery_soc_lut) / sizeof(battery_soc_lut[0])) - 1].soc_pct;
    }

    uint8_t lut_size = (uint8_t)(sizeof(battery_soc_lut) / sizeof(battery_soc_lut[0]));
    for (uint8_t i = 0; i < (lut_size - 1u); ++i) {
        const BatterySocPoint_t *low = &battery_soc_lut[i];
        const BatterySocPoint_t *high = &battery_soc_lut[i + 1u];

        if (compensated_mv >= high->voltage_mv && compensated_mv <= low->voltage_mv) {
            int32_t span = (int32_t)low->voltage_mv - (int32_t)high->voltage_mv;
            if (span <= 0) {
                return low->soc_pct;
            }

            int32_t delta = (int32_t)low->soc_pct - (int32_t)high->soc_pct;
            int32_t offset = (int32_t)compensated_mv - (int32_t)high->voltage_mv;
            int32_t interpolated = (int32_t)high->soc_pct + ((offset * delta) / span);
            return (uint8_t)interpolated;
        }
    }

    return battery_soc_lut[0].soc_pct;
}

float Battery_Get_MAh_Consumed(void) {
    return mah_consumed_model;
}

void Battery_Load_State_From_EEPROM(void) {
    /* Battery percentage must always come from the current voltage reading, never
       from a stale stored SOC value. Only restore the model-based consumed-charge estimate. */
    uint16_t saved_mah_q10 = EepromSettings_LoadBatteryConsumedQ10();

    battery_soc_pct = 0U;
    mah_consumed_model = (float)saved_mah_q10 / 10.0f;
}

EepromSettingsStatus_t Battery_Save_State_To_EEPROM(void) {
    uint16_t mah_q10 = (uint16_t)(mah_consumed_model * 10.0f + 0.5f);
    if (mah_q10 > 900U) {
        mah_q10 = 900U;
    }

    return EepromSettings_SaveBatteryConsumedQ10(mah_q10);
}

static void Battery_Require_Persist(void) {
    if (Battery_Save_State_To_EEPROM() != EEPROM_SETTINGS_OK) {
        HALT_WITH_ERROR(ERR_EEPROM_STORAGE);
    }
}

void Battery_Reset_Fusion_Consumed(void) {
    mah_consumed_model = 0.0f;
    last_mah_update_ms = 0;
    battery_was_full_and_charging = 0;
    Battery_Require_Persist();
}

void Battery_Update_Estimate(uint32_t battery_mv_mV) {
    uint32_t now_ms = HAL_GetTick();
    if (last_mah_update_ms == 0) {
        last_mah_update_ms = now_ms;
    }

    uint32_t dt_ms = now_ms - last_mah_update_ms;
    if (dt_ms > 0) {
        float dt_h = (float)dt_ms / 1000.0f / 3600.0f;
        float state_current_ma = 0.0f;

        switch (current_state) {
            case STATE_OFF:
                state_current_ma = BATTERY_STATE_OFF_CURRENT_MA;
                break;
            case STATE_ACTIVE:
                state_current_ma = BATTERY_STATE_ACTIVE_CURRENT_MA;
                break;
            case STATE_CHARGING:
                /* Charging voltage is not a trustworthy SOC indicator and can spike
                   above the real battery level while the external supply is present.
                   Keep the consumed-charge estimate conservative instead of letting a charger
                   step the reported SOC directly to 100%. */
                state_current_ma = 0.0f;
                break;
            default:
                state_current_ma = 0.0f;
                break;
        }

        mah_consumed_model += state_current_ma * dt_h;
        if (mah_consumed_model < 0.0f) {
            mah_consumed_model = 0.0f;
        }
    }
    last_mah_update_ms = now_ms;

    if (HAL_GPIO_ReadPin(USB_PORT, USB_PIN) == GPIO_PIN_SET) {
        if (battery_mv_mV >= BATTERY_FULL_MV) {
            battery_was_full_and_charging = 1;
        }
    } else if (battery_was_full_and_charging && battery_mv_mV >= BATTERY_CHARGE_RESET_MV) {
        Battery_Reset_Fusion_Consumed();
    }

    Battery_Require_Persist();
}

uint8_t Battery_Get_Fused_SOC(uint32_t mv, int32_t temp_c) {
    uint8_t v_soc = Battery_Get_SOC_From_Voltage(mv, temp_c);
    if (mv >= 4100U) {
        v_soc = 100U;
    } else if (mv >= 4000U) {
        v_soc = BATTERY_FULL_SOH_SOC_PCT;
    }

    float consumed_charge_soc = 100.0f - ((mah_consumed_model / BATTERY_CAPACITY_MAH) * 100.0f);
    if (consumed_charge_soc < 0.0f) {
        consumed_charge_soc = 0.0f;
    }
    if (consumed_charge_soc > 100.0f) {
        consumed_charge_soc = 100.0f;
    }

    float fused = (BATTERY_VOLTAGE_SOC_WEIGHT * (float)v_soc) + (BATTERY_CONSUMED_SOC_WEIGHT * consumed_charge_soc);
    if (current_state == STATE_CHARGING) {
        /* Charger voltage is usually higher than the real cell voltage, so the
           raw terminal voltage cannot be trusted as a full indicator while USB is present.
           Cap charge estimation so it cannot jump straight to 100% from a charger spike. */
        if (v_soc > BATTERY_CHARGING_SOC_CAP_PCT) {
            v_soc = BATTERY_CHARGING_SOC_CAP_PCT;
        }
        if (consumed_charge_soc > BATTERY_CHARGING_SOC_CAP_PCT) {
            consumed_charge_soc = BATTERY_CHARGING_SOC_CAP_PCT;
        }
        fused = (BATTERY_VOLTAGE_SOC_WEIGHT * (float)v_soc) + (BATTERY_CONSUMED_SOC_WEIGHT * consumed_charge_soc);
        if (mv >= BATTERY_FULL_MV && v_soc >= BATTERY_CHARGING_SOC_CAP_PCT && consumed_charge_soc >= BATTERY_CHARGING_SOC_CAP_PCT) {
            fused = 100.0f;
        }
    }
    if (fused < 0.0f) {
        fused = 0.0f;
    }
    if (fused > 100.0f) {
        fused = 100.0f;
    }
    return (uint8_t)fused;
}

/* Flashes low-battery warning then requests a clean shutdown. */
void Handle_Critical_Battery(void) {
    Flash_Low_Battery_Warning(LOW_BATT_WARN_MS);
    current_state = STATE_OFF;
}
