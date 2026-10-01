/*
 * led.h — LED display: battery level, charging status, volume feedback
 *
 * Author: Luca Obwegs
 */
#ifndef INC_LED_H_
#define INC_LED_H_

#include "main.h"

// --- Battery level display thresholds (SOC %) ---
#define BAT_SOC_FULL_PCT    90   // all 3 LEDs
#define BAT_SOC_66_PCT      66   // 2 LEDs (charging display only)
#define BAT_SOC_LOW_PCT     30   // 1 LED
#define BAT_SOC_HYST_PCT      4   // hysteresis around charging-display thresholds

typedef enum {
    BATLVL_LOW = 0,
    BATLVL_MID,
    BATLVL_HIGH,
    BATLVL_FULL
} BatteryLevel_t;

void Flash_Low_Battery_Warning(uint32_t duration_ms);
void Display_Battery_Level_Static(uint8_t soc_pct);
void Display_Charging_Status(uint8_t soc_pct);
void Display_Volume_LED(uint8_t level);

#endif /* INC_LED_H_ */
