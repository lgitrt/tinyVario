/*
 * eeprom_settings.c — Versioned and checksummed Data EEPROM records
 */
#include "eeprom_settings.h"

#ifndef EEPROM_SETTINGS_HOST_TEST
#include "main.h"

_Static_assert(EEPROM_SETTINGS_BASE_ADDR == DATA_EEPROM_BASE,
               "settings base must match the selected STM32 Data EEPROM map");
_Static_assert(EEPROM_SETTINGS_END_ADDR == DATA_EEPROM_END,
               "settings end must match the selected STM32 Data EEPROM map");
_Static_assert(EEPROM_BATTERY_RECORD_ADDR + 4UL <= EEPROM_SETTINGS_END_ADDR,
               "settings records must fit in Data EEPROM");
#endif

#define EEPROM_VOLUME_VERSION  0xA1U
#define EEPROM_BATTERY_VERSION 0xB1U
#define EEPROM_VOLUME_RECORD_SIZE  4U
#define EEPROM_BATTERY_RECORD_SIZE 5U

volatile uint32_t dbg_eeprom_settings_write_errors = 0U;

#ifdef EEPROM_SETTINGS_HOST_TEST
static uint8_t test_data_eeprom[EEPROM_SETTINGS_END_ADDR - EEPROM_SETTINGS_BASE_ADDR + 1UL];
static int32_t test_writes_before_failure = -1;
static uint8_t test_storage_initialized = 0U;
#endif

static uint8_t Eeprom_ReadByte(uint32_t address)
{
#ifdef EEPROM_SETTINGS_HOST_TEST
    if (!test_storage_initialized) {
        EepromSettings_TestReset();
    }
    return test_data_eeprom[address - EEPROM_SETTINGS_BASE_ADDR];
#else
    return *(__IO uint8_t *)address;
#endif
}

static uint16_t Eeprom_Crc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;
    for (uint8_t i = 0U; i < length; ++i) {
        crc ^= (uint16_t)data[i] << 8U;
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            crc = (crc & 0x8000U) ? (uint16_t)((crc << 1U) ^ 0x1021U)
                                  : (uint16_t)(crc << 1U);
        }
    }
    return crc;
}

static void Eeprom_ReadRecord(uint32_t address, uint8_t *record, uint8_t length)
{
    for (uint8_t i = 0U; i < length; ++i) {
        record[i] = Eeprom_ReadByte(address + i);
    }
}

static EepromSettingsStatus_t Eeprom_WriteRecord(uint32_t address,
                                                  const uint8_t *record,
                                                  uint8_t length)
{
    uint8_t changed = 0U;
    for (uint8_t i = 0U; i < length; ++i) {
        if (Eeprom_ReadByte(address + i) != record[i]) {
            changed = 1U;
            break;
        }
    }
    if (!changed) {
        return EEPROM_SETTINGS_OK;
    }

#ifdef EEPROM_SETTINGS_HOST_TEST
    for (uint8_t i = 0U; i < length; ++i) {
        uint32_t offset = address + i - EEPROM_SETTINGS_BASE_ADDR;
        if (test_data_eeprom[offset] == record[i]) {
            continue;
        }
        if (test_writes_before_failure == 0) {
            dbg_eeprom_settings_write_errors++;
            return EEPROM_SETTINGS_IO_ERROR;
        }
        if (test_writes_before_failure > 0) {
            test_writes_before_failure--;
        }
        test_data_eeprom[offset] = record[i];
    }
#else
    HAL_StatusTypeDef status = HAL_FLASHEx_DATAEEPROM_Unlock();
    if (status != HAL_OK) {
        dbg_eeprom_settings_write_errors++;
        return EEPROM_SETTINGS_IO_ERROR;
    }

    for (uint8_t i = 0U; i < length; ++i) {
        if (Eeprom_ReadByte(address + i) == record[i]) {
            continue;
        }
        status = HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_BYTE,
                                                address + i,
                                                (uint32_t)record[i]);
        if (status != HAL_OK) {
            break;
        }
    }

    HAL_StatusTypeDef lock_status = HAL_FLASHEx_DATAEEPROM_Lock();
    if (status == HAL_OK && lock_status != HAL_OK) {
        status = lock_status;
    }
    if (status != HAL_OK) {
        dbg_eeprom_settings_write_errors++;
        return EEPROM_SETTINGS_IO_ERROR;
    }
#endif

    return EEPROM_SETTINGS_OK;
}

static uint8_t Eeprom_ValidateRecord(const uint8_t *record, uint8_t payload_length)
{
    uint16_t stored_crc = (uint16_t)record[payload_length] |
                          ((uint16_t)record[payload_length + 1U] << 8U);
    return stored_crc == Eeprom_Crc16(record, payload_length);
}

uint8_t EepromSettings_LoadVolume(void)
{
    uint8_t record[EEPROM_VOLUME_RECORD_SIZE];
    Eeprom_ReadRecord(EEPROM_VOLUME_RECORD_ADDR, record, EEPROM_VOLUME_RECORD_SIZE);

    if (record[0] != EEPROM_VOLUME_VERSION ||
        record[1] < 1U || record[1] > 3U ||
        !Eeprom_ValidateRecord(record, 2U)) {
        return 3U;
    }
    return record[1];
}

EepromSettingsStatus_t EepromSettings_SaveVolume(uint8_t volume)
{
    if (volume < 1U || volume > 3U) {
        return EEPROM_SETTINGS_INVALID_INPUT;
    }

    uint8_t record[EEPROM_VOLUME_RECORD_SIZE] = {
        EEPROM_VOLUME_VERSION,
        volume,
        0U,
        0U
    };
    uint16_t crc = Eeprom_Crc16(record, 2U);
    record[2] = (uint8_t)(crc & 0xFFU);
    record[3] = (uint8_t)(crc >> 8U);
    return Eeprom_WriteRecord(EEPROM_VOLUME_RECORD_ADDR, record,
                              EEPROM_VOLUME_RECORD_SIZE);
}

uint16_t EepromSettings_LoadBatteryConsumedQ10(void)
{
    uint8_t record[EEPROM_BATTERY_RECORD_SIZE];
    Eeprom_ReadRecord(EEPROM_BATTERY_RECORD_ADDR, record, EEPROM_BATTERY_RECORD_SIZE);

    uint16_t consumed_q10 = (uint16_t)record[1] |
                            ((uint16_t)record[2] << 8U);
    if (record[0] != EEPROM_BATTERY_VERSION ||
        consumed_q10 > EEPROM_BATTERY_MAX_CONSUMED_Q10 ||
        !Eeprom_ValidateRecord(record, 3U)) {
        return 0U;
    }
    return consumed_q10;
}

EepromSettingsStatus_t EepromSettings_SaveBatteryConsumedQ10(uint16_t consumed_q10)
{
    if (consumed_q10 > EEPROM_BATTERY_MAX_CONSUMED_Q10) {
        return EEPROM_SETTINGS_INVALID_INPUT;
    }

    uint8_t record[EEPROM_BATTERY_RECORD_SIZE] = {
        EEPROM_BATTERY_VERSION,
        (uint8_t)(consumed_q10 & 0xFFU),
        (uint8_t)(consumed_q10 >> 8U),
        0U,
        0U
    };
    uint16_t crc = Eeprom_Crc16(record, 3U);
    record[3] = (uint8_t)(crc & 0xFFU);
    record[4] = (uint8_t)(crc >> 8U);
    return Eeprom_WriteRecord(EEPROM_BATTERY_RECORD_ADDR, record,
                              EEPROM_BATTERY_RECORD_SIZE);
}

#ifdef EEPROM_SETTINGS_HOST_TEST
void EepromSettings_TestReset(void)
{
    for (uint32_t i = 0U; i < sizeof(test_data_eeprom); ++i) {
        test_data_eeprom[i] = 0xFFU;
    }
    dbg_eeprom_settings_write_errors = 0U;
    test_writes_before_failure = -1;
    test_storage_initialized = 1U;
}

void EepromSettings_TestFailAfterWrites(int32_t successful_writes)
{
    test_writes_before_failure = successful_writes;
}

void EepromSettings_TestFlipBit(uint32_t address, uint8_t bit)
{
    if (!test_storage_initialized) {
        EepromSettings_TestReset();
    }
    test_data_eeprom[address - EEPROM_SETTINGS_BASE_ADDR] ^= (uint8_t)(1U << bit);
}
#endif
