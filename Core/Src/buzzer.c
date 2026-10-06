/*
 * buzzer.c — Audio engine: beep pattern, pitch, cadence, PAM8904 volume control
 *
 * Author: Luca Obwegs
 */
#include "buzzer.h"

extern TIM_HandleTypeDef htim2;

uint8_t g_volume   = 3;   // default max; overwritten by Load_Volume() on boot
uint8_t is_beeping = 0;

static uint32_t last_beep_tick  = 0;
static uint16_t beep_duration   = 200;
static uint16_t silent_duration = 200;
/* Durations locked at each transition — prevents mid-period velocity changes from cutting the period short */
static uint16_t active_beep_dur   = 200;
static uint16_t active_silent_dur = 200;

/* PAM8904 EN truth table: 0=shutdown, 1=low, 2=mid, 3=high */
void Set_Volume(uint8_t level) {
    HAL_GPIO_WritePin(BUZ_EN1_GPIO_Port, BUZ_EN1_Pin,
        (level == 2 || level == 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(BUZ_EN2_GPIO_Port, BUZ_EN2_Pin,
        (level == 1 || level == 3) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

uint8_t Load_Volume(void) {
    return EepromSettings_LoadVolume();
}

EepromSettingsStatus_t Save_Volume(uint8_t vol) {
    return EepromSettings_SaveVolume(vol);
}

void Set_Buzzer_Frequency(uint32_t frequency) {
    if (frequency < 100) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
        return;
    }
    uint32_t timer_clk = HAL_RCC_GetPCLK1Freq();
    if ((RCC->CFGR & RCC_CFGR_PPRE1) != RCC_CFGR_PPRE1_DIV1) timer_clk *= 2;
    uint32_t arr = (timer_clk / frequency) - 1;
    if (arr > 0xFFFF) arr = 0xFFFF;
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (arr + 1) / 2);
}

void Update_Buzzer(int32_t v_vel) {
    static uint32_t last_applied_freq = 0;
    uint32_t now = HAL_GetTick();

    if (v_vel >= LIFT_THRESHOLD) {
        uint32_t target_freq = 4000;
        uint16_t total_cycle = BEEP_CYCLE_MAX_MS - (v_vel / BEEP_CYCLE_SLOPE_MS);
        if (total_cycle < BEEP_CYCLE_MIN_MS) total_cycle = BEEP_CYCLE_MIN_MS;
        beep_duration   = total_cycle / 2;
        silent_duration = total_cycle / 2;

        if (!is_beeping && (now - last_beep_tick >= active_silent_dur)) {
            Set_Buzzer_Frequency(target_freq);
            last_applied_freq = target_freq;
            is_beeping     = 1;
            last_beep_tick = now;
            active_beep_dur = beep_duration;    /* lock beep length at start of beep */
        } else if (is_beeping && (now - last_beep_tick >= active_beep_dur)) {
            Set_Buzzer_Frequency(0);
            last_applied_freq = 0;
            is_beeping     = 0;
            last_beep_tick = now;
            active_silent_dur = silent_duration; /* lock silence length at start of silence */
        }
    } else if (v_vel <= SINK_THRESHOLD) {
        Set_Buzzer_Frequency(SINK_FREQUENCY);
        last_applied_freq = SINK_FREQUENCY;
        is_beeping = 1;
    } else {
        /* Dead zone: let any in-progress beep complete its locked duration before stopping */
        if (is_beeping && (now - last_beep_tick < active_beep_dur)) {
            /* still within the beep's natural duration — don't cut it */
        } else {
            if (last_applied_freq != 0) {
                Set_Buzzer_Frequency(0);
                last_applied_freq = 0;
            }
            is_beeping = 0;
        }
    }
}
