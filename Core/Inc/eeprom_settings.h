/*
 * eeprom_settings.h — Versioned persistent settings for STM32L031 Data EEPROM
 */
#ifndef INC_EEPROM_SETTINGS_H_
#define INC_EEPROM_SETTINGS_H_

#include <stdint.h>

#define EEPROM_SETTINGS_BASE_ADDR       0x08080000UL
#define EEPROM_SETTINGS_END_ADDR        0x080803FFUL
#define EEPROM_VOLUME_RECORD_ADDR       EEPROM_SETTINGS_BASE_ADDR
#define EEPROM_BATTERY_RECORD_ADDR     (EEPROM_VOLUME_RECORD_ADDR + 4UL)
#define EEPROM_SETTINGS_USED_BYTES       9U
#define EEPROM_BATTERY_MAX_CONSUMED_Q10 900U

typedef enum {
    EEPROM_SETTINGS_OK = 0,
    EEPROM_SETTINGS_INVALID_INPUT,
    EEPROM_SETTINGS_IO_ERROR
} EepromSettingsStatus_t;

extern volatile uint32_t dbg_eeprom_settings_write_errors;

uint8_t EepromSettings_LoadVolume(void);
EepromSettingsStatus_t EepromSettings_SaveVolume(uint8_t volume);
uint16_t EepromSettings_LoadBatteryConsumedQ10(void);
EepromSettingsStatus_t EepromSettings_SaveBatteryConsumedQ10(uint16_t consumed_q10);

#ifdef EEPROM_SETTINGS_HOST_TEST
void EepromSettings_TestReset(void);
void EepromSettings_TestFailAfterWrites(int32_t successful_writes);
void EepromSettings_TestFlipBit(uint32_t address, uint8_t bit);
#endif

#endif /* INC_EEPROM_SETTINGS_H_ */
