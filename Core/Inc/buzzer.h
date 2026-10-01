/*
 * buzzer.h — Audio engine: beep pattern, pitch, cadence, PAM8904 volume control
 *
 * Author: Luca Obwegs
 */
#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "main.h"
#include "filter_tuning.h"  /* LIFT_THRESHOLD, SINK_THRESHOLD, BEEP_CYCLE_* */

// --- PAM8904 volume / EEPROM ---
#define EEPROM_VOLUME_ADDR  0x08080000U   // STM32L031 Data EEPROM start

extern uint8_t g_volume;    // 1=low, 2=mid, 3=high; updated by volume cycle
extern uint8_t is_beeping;

void Set_Buzzer_Frequency(uint32_t frequency);
void Update_Buzzer(int32_t v_vel);
void Set_Volume(uint8_t level);
uint8_t  Load_Volume(void);
void Save_Volume(uint8_t vol);

#endif /* INC_BUZZER_H_ */
