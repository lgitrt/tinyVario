/*
 * led.c — LED display: battery level, charging status, volume feedback
 *
 * Author: Luca Obwegs
 */
#include "led.h"

static BatteryLevel_t current_batt_level = BATLVL_LOW;

/* Flashes BATled1 at ~2 Hz for duration_ms — low-battery warning. */
void Flash_Low_Battery_Warning(uint32_t duration_ms) {
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < duration_ms) {
        HAL_GPIO_WritePin(GPIOA, BATled1_Pin, GPIO_PIN_SET);
        HAL_Delay(250);
        HAL_GPIO_WritePin(GPIOA, BATled1_Pin, GPIO_PIN_RESET);
        HAL_Delay(250);
    }
}

/* Static power-on battery readout. Assumes all 3 LEDs are already ON;
   turns off only the ones that don't belong at the current SOC level. */
void Display_Battery_Level_Static(uint8_t soc_pct) {
    if (soc_pct < BAT_SOC_LOW_PCT) {
        HAL_GPIO_WritePin(GPIOA, BATled2_Pin | BATled3_Pin, GPIO_PIN_RESET);
    } else if (soc_pct < BAT_SOC_FULL_PCT) {
        HAL_GPIO_WritePin(GPIOA, BATled3_Pin, GPIO_PIN_RESET);
    }
    /* else >= BAT_SOC_FULL_PCT: leave all 3 on */
}

/* Animated charging display — call repeatedly from the charging state loop. */
void Display_Charging_Status(uint8_t soc_pct) {
    uint8_t toggle = (HAL_GetTick() % 1000) < 500;

    switch (current_batt_level) {
        case BATLVL_LOW:
            if (soc_pct > BAT_SOC_LOW_PCT + BAT_SOC_HYST_PCT)  current_batt_level = BATLVL_MID;
            break;
        case BATLVL_MID:
            if      (soc_pct > BAT_SOC_66_PCT  + BAT_SOC_HYST_PCT)  current_batt_level = BATLVL_HIGH;
            else if (soc_pct < BAT_SOC_LOW_PCT - BAT_SOC_HYST_PCT)  current_batt_level = BATLVL_LOW;
            break;
        case BATLVL_HIGH:
            if      (soc_pct > BAT_SOC_FULL_PCT + BAT_SOC_HYST_PCT) current_batt_level = BATLVL_FULL;
            else if (soc_pct < BAT_SOC_66_PCT   - BAT_SOC_HYST_PCT) current_batt_level = BATLVL_MID;
            break;
        case BATLVL_FULL:
            if (soc_pct < BAT_SOC_FULL_PCT - BAT_SOC_HYST_PCT) current_batt_level = BATLVL_HIGH;
            break;
    }

    HAL_GPIO_WritePin(GPIOA, BATled1_Pin | BATled2_Pin | BATled3_Pin, GPIO_PIN_RESET);

    switch (current_batt_level) {
        case BATLVL_LOW:
            if (toggle) HAL_GPIO_WritePin(GPIOA, BATled1_Pin, GPIO_PIN_SET);
            break;
        case BATLVL_MID:
            HAL_GPIO_WritePin(GPIOA, BATled1_Pin, GPIO_PIN_SET);
            if (toggle) HAL_GPIO_WritePin(GPIOA, BATled2_Pin, GPIO_PIN_SET);
            break;
        case BATLVL_HIGH:
            HAL_GPIO_WritePin(GPIOA, BATled1_Pin | BATled2_Pin, GPIO_PIN_SET);
            if (toggle) HAL_GPIO_WritePin(GPIOA, BATled3_Pin, GPIO_PIN_SET);
            break;
        case BATLVL_FULL:
            HAL_GPIO_WritePin(GPIOA, BATled1_Pin | BATled2_Pin | BATled3_Pin, GPIO_PIN_SET);
            break;
    }
}

/* Shows volume level on LEDs for 1 second then clears. level 1=1 LED … 3=3 LEDs. */
void Display_Volume_LED(uint8_t level) {
    uint32_t mask = 0;
    if (level >= 1) mask |= BATled1_Pin;
    if (level >= 2) mask |= BATled2_Pin;
    if (level >= 3) mask |= BATled3_Pin;
    HAL_GPIO_WritePin(GPIOA, BATled1_Pin | BATled2_Pin | BATled3_Pin, GPIO_PIN_RESET);
    if (mask) HAL_GPIO_WritePin(GPIOA, mask, GPIO_PIN_SET);
    HAL_Delay(500);  /* 500 ms: visible but short enough to limit KF stale-state gap */
    HAL_GPIO_WritePin(GPIOA, BATled1_Pin | BATled2_Pin | BATled3_Pin, GPIO_PIN_RESET);
}
