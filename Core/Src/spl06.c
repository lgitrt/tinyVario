/*
 * spl06.c — SPL06 barometer driver: calibration readout, pressure/temperature compensation
 *
 * Author: Luca Obwegs
 */
#include "spl06.h"

static SPL06_Calib_t calib;

// Scale factor for 16x oversampling is 253952
// We will use 64-bit math for the compensation to avoid rounding errors
#define K_SCALE 7864320

HAL_StatusTypeDef SPL06_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t coef[18];
    if (HAL_I2C_Mem_Read(hi2c, SPL06_I2C_ADDR, 0x10, 1, coef, 18, 100) != HAL_OK)
        return HAL_ERROR;

    // Same coefficient parsing as before...
    calib.c0  = (coef[0] << 4) | (coef[1] >> 4);
    if (calib.c0 & 0x0800) calib.c0 |= 0xFFFFF000;
    calib.c1  = ((coef[1] & 0x0F) << 8) | coef[2];
    if (calib.c1 & 0x0800) calib.c1 |= 0xFFFFF000;
    calib.c00 = ((uint32_t)coef[3] << 12) | ((uint32_t)coef[4] << 4) | (coef[5] >> 4);
    if (calib.c00 & 0x080000) calib.c00 |= 0xFFF00000;
    calib.c10 = (((uint32_t)coef[5] & 0x0F) << 16) | ((uint32_t)coef[6] << 8) | coef[7];
    if (calib.c10 & 0x080000) calib.c10 |= 0xFFF00000;

    // c01, c11, c20, c21, c30 are all signed 16-bit values per the datasheet -
    // these were missing sign extension, which silently turned any negative
    // coefficient into a huge positive one and threw a large constant bias
    // into every pressure reading.
    calib.c01 = (coef[8] << 8) | coef[9];
    if (calib.c01 & 0x8000) calib.c01 |= 0xFFFF0000;
    calib.c11 = (coef[10] << 8) | coef[11];
    if (calib.c11 & 0x8000) calib.c11 |= 0xFFFF0000;
    calib.c20 = (coef[12] << 8) | coef[13];
    if (calib.c20 & 0x8000) calib.c20 |= 0xFFFF0000;
    calib.c21 = (coef[14] << 8) | coef[15];
    if (calib.c21 & 0x8000) calib.c21 |= 0xFFFF0000;
    calib.c30 = (coef[16] << 8) | coef[17];
    if (calib.c30 & 0x8000) calib.c30 |= 0xFFFF0000;

    uint8_t init_cmds[4][2] = {
        {0x06, 0x33}, // PRS_CFG   - 8 meas/sec, 8x oversampling
        {0x07, 0xB3}, // TMP_CFG   - external sensor, 8 meas/sec, 8x oversampling
        {0x08, 0x07}, // MEAS_CFG  - continuous pressure+temperature
        {0x09, 0x00}  // CFG_REG   - no result bit-shift
    };
    for (int i = 0; i < 4; i++) {
        if (HAL_I2C_Mem_Write(hi2c, SPL06_I2C_ADDR, init_cmds[i][0], 1, &init_cmds[i][1], 1, 10) != HAL_OK)
            return HAL_ERROR;
    }

    return HAL_OK;
}

int32_t SPL06_GetTemperature(I2C_HandleTypeDef *hi2c) {
    uint8_t data[3];
    HAL_I2C_Mem_Read(hi2c, SPL06_I2C_ADDR, 0x03, 1, data, 3, 10);
    int32_t raw_t = (data[0] << 16) | (data[1] << 8) | data[2];
    if (raw_t & 0x800000) raw_t |= 0xFF000000;

    // Temperature scaled value (Traw_sc)
    calib.last_t_raw_sc = raw_t; // We keep the raw value for the scale division later

    // Compansated Temp = c0*0.5 + c1*Traw_sc
    // To keep it fixed point, we calculate mC (milli-Celsius)
    int64_t t_comp = ((int64_t)calib.c0 * 500) + (((int64_t)calib.c1 * 1000 * raw_t) / K_SCALE);
    return (int32_t)t_comp;
}

int32_t SPL06_GetPressure(I2C_HandleTypeDef *hi2c) {
    uint8_t data[3];
    HAL_I2C_Mem_Read(hi2c, SPL06_I2C_ADDR, 0x00, 1, data, 3, 10);
    int32_t raw_p = (data[0] << 16) | (data[1] << 8) | data[2];
    if (raw_p & 0x800000) raw_p |= 0xFF000000;

    // Use 64-bit for intermediate pressure terms to avoid overflow
    int64_t pr_raw_sc = (int64_t)raw_p;
    int64_t tr_raw_sc = (int64_t)calib.last_t_raw_sc;

    // Simplified terms to match datasheet formula but in fixed-point
    // Formula: Pcomp = c00 + pr_sc*(c10+pr_sc*(c20+pr_sc*c30)) + tr_sc*c01 + tr_sc*pr_sc*(c11+pr_sc*c21)

    // We must divide by K_SCALE at each stage where pr_sc or tr_sc is multiplied
    int64_t term1 = (int64_t)calib.c00;
    int64_t term2 = (pr_raw_sc * (calib.c10 + (pr_raw_sc * (calib.c20 + (pr_raw_sc * calib.c30) / K_SCALE)) / K_SCALE)) / K_SCALE;
    int64_t term3 = (tr_raw_sc * calib.c01) / K_SCALE;
    // Break the division into two steps to prevent overflow and stay efficient
    int64_t internal_term = (tr_raw_sc * pr_raw_sc) / K_SCALE;
    int64_t term4 = (internal_term * (calib.c11 + (pr_raw_sc * calib.c21) / K_SCALE)) / K_SCALE;

    int32_t p_pa = (int32_t)(term1 + term2 + term3 + term4);
    return p_pa; // Returns Pressure in Pascals
}

// bit4 = PRS_RDY, bit5 = TMP_RDY (verify against your SPL06 datasheet revision)
uint8_t SPL06_DataReady(I2C_HandleTypeDef *hi2c, uint8_t mask) {
    uint8_t meas_cfg;
    if (HAL_I2C_Mem_Read(hi2c, SPL06_I2C_ADDR, 0x08, 1, &meas_cfg, 1, 10) != HAL_OK)
        return 0;
    return (meas_cfg & mask) ? 1 : 0;
}

HAL_StatusTypeDef SPL06_EnterStandby(I2C_HandleTypeDef *hi2c)
{
    uint8_t standby = 0x00;
    return HAL_I2C_Mem_Write(hi2c, SPL06_I2C_ADDR, SPL06_MEAS_CFG,
                              I2C_MEMADD_SIZE_8BIT, &standby, 1, 10);
}

uint8_t SPL06_IsInStandby(I2C_HandleTypeDef *hi2c) {
    uint8_t meas_cfg = 0xFF;
    if (HAL_I2C_Mem_Read(hi2c, SPL06_I2C_ADDR, SPL06_MEAS_CFG, 1, &meas_cfg, 1, 10) != HAL_OK)
        return 0; // couldn't even read it back - can't confirm, treat as failed
    return (meas_cfg & 0x07) == 0x00; // MEAS_CTRL[2:0] = 000 -> standby, no background sampling
}
