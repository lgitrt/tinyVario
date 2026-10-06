/*
 * test_eeprom_settings.c — Host regressions for volume/battery EEPROM storage.
 */
#include "test_util.h"
#include "eeprom_settings.h"

static void test_erased_and_legacy_data_defaults(void)
{
    EepromSettings_TestReset();
    CHECK(EepromSettings_LoadVolume() == 3U,
          "erased volume storage must use the documented maximum-volume default");
    CHECK(EepromSettings_LoadBatteryConsumedQ10() == 0U,
          "erased battery storage must start with zero consumed estimate");

    EepromSettings_TestReset();
    EepromSettings_TestFlipBit(EEPROM_SETTINGS_BASE_ADDR, 0U);
    CHECK(EepromSettings_LoadVolume() == 3U,
          "overlapping legacy volume/battery data must not be accepted as a versioned volume record");
    CHECK(EepromSettings_LoadBatteryConsumedQ10() == 0U,
          "legacy battery bytes must not be accepted as a versioned battery record");
}

static void test_records_are_independent_after_restart(void)
{
    EepromSettings_TestReset();
    CHECK(EEPROM_VOLUME_RECORD_ADDR + 3U < EEPROM_BATTERY_RECORD_ADDR,
          "the volume and battery records must occupy disjoint EEPROM bytes");
    CHECK(EEPROM_BATTERY_RECORD_ADDR + 4U <= EEPROM_SETTINGS_END_ADDR,
          "both records must remain inside the STM32L031 Data EEPROM range");
    CHECK(EEPROM_SETTINGS_USED_BYTES == 9U,
          "record map size must match the current volume and battery records");

    CHECK(EepromSettings_SaveVolume(2U) == EEPROM_SETTINGS_OK,
          "saving volume should succeed");
    CHECK(EepromSettings_SaveBatteryConsumedQ10(275U) == EEPROM_SETTINGS_OK,
          "saving the consumed-charge estimate should succeed");
    CHECK(EepromSettings_LoadVolume() == 2U,
          "saved volume must survive a fresh load");
    CHECK(EepromSettings_LoadBatteryConsumedQ10() == 275U,
          "saved battery estimate must survive a fresh load");

    CHECK(EepromSettings_SaveVolume(1U) == EEPROM_SETTINGS_OK,
          "changing volume should succeed");
    CHECK(EepromSettings_LoadBatteryConsumedQ10() == 275U,
          "changing volume must not alter battery state");
    CHECK(EepromSettings_SaveBatteryConsumedQ10(325U) == EEPROM_SETTINGS_OK,
          "changing battery estimate should succeed");
    CHECK(EepromSettings_LoadVolume() == 1U,
          "changing battery state must not alter volume");
    CHECK(EepromSettings_LoadBatteryConsumedQ10() == 325U,
          "the latest battery state must be restored after restart");
}

static void test_single_bit_corruption_uses_defaults(void)
{
    for (uint8_t i = 0U; i < 4U; ++i) {
        EepromSettings_TestReset();
        CHECK(EepromSettings_SaveVolume(2U) == EEPROM_SETTINGS_OK,
              "volume corruption precondition: save succeeds");
        EepromSettings_TestFlipBit(EEPROM_VOLUME_RECORD_ADDR + i, 0U);
        CHECK(EepromSettings_LoadVolume() == 3U,
              "a corrupted volume record must use the default");
    }

    for (uint8_t i = 0U; i < 5U; ++i) {
        EepromSettings_TestReset();
        CHECK(EepromSettings_SaveBatteryConsumedQ10(250U) == EEPROM_SETTINGS_OK,
              "battery corruption precondition: save succeeds");
        EepromSettings_TestFlipBit(EEPROM_BATTERY_RECORD_ADDR + i, 0U);
        CHECK(EepromSettings_LoadBatteryConsumedQ10() == 0U,
              "a corrupted battery record must use zero consumed estimate");
    }
}

static void test_interrupted_write_and_invalid_inputs(void)
{
    EepromSettings_TestReset();
    EepromSettings_TestFailAfterWrites(2);
    CHECK(EepromSettings_SaveBatteryConsumedQ10(250U) == EEPROM_SETTINGS_IO_ERROR,
          "a failed EEPROM program operation must be reported");
    CHECK(dbg_eeprom_settings_write_errors == 1U,
          "EEPROM program failure must remain visible in the debugger counter");
    CHECK(EepromSettings_LoadBatteryConsumedQ10() == 0U,
          "an interrupted record update must fail validation and use the default");

    EepromSettings_TestFailAfterWrites(-1);
    CHECK(EepromSettings_SaveVolume(0U) == EEPROM_SETTINGS_INVALID_INPUT,
          "volume outside the supported range must be rejected");
    CHECK(EepromSettings_SaveBatteryConsumedQ10(901U) == EEPROM_SETTINGS_INVALID_INPUT,
          "battery estimate outside the supported range must be rejected");
    CHECK(EepromSettings_LoadVolume() == 3U,
          "invalid volume input must not alter stored settings");
    CHECK(EepromSettings_LoadBatteryConsumedQ10() == 0U,
          "invalid battery input must not alter stored settings");
}

int main(void)
{
    test_erased_and_legacy_data_defaults();
    test_records_are_independent_after_restart();
    test_single_bit_corruption_uses_defaults();
    test_interrupted_write_and_invalid_inputs();
    TEST_SUMMARY();
}
