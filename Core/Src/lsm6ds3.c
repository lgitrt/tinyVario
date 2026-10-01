/*
 * lsm6ds3.c — LSM6DS3 IMU driver: init, DMA/poll read, raw-to-fixed-point conversion
 *
 * Author: Luca Obwegs
 */
#include "lsm6ds3.h"

// Sensitivities for Fixed-Point math
// @ +/- 2g: 0.061 mg/LSB
// @ +/- 245dps: 8.75 mdps/LSB
#define ACCEL_SENSITIVITY_2G    61   // x1000
#define GYRO_SENSITIVITY_245    8750 // x1000 (millidegrees)

HAL_StatusTypeDef LSM6DS3_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t config[2];

    // 1. Accel: 26Hz (0x20), +/- 2g (0x00), BW 400Hz (0x00) -> 0x20
    config[0] = LSM6DS3_CTRL1_XL;
    config[1] = 0x20;
    if(HAL_I2C_Master_Transmit(hi2c, LSM6DS3_ADDR, config, 2, 10) != HAL_OK) return HAL_ERROR;

    // 2. Gyro: 26Hz (0x20), 245 dps (0x00) -> 0x20
    config[0] = LSM6DS3_CTRL2_G;
    config[1] = 0x20;
    if (HAL_I2C_Master_Transmit(hi2c, LSM6DS3_ADDR, config, 2, 10) != HAL_OK) return HAL_ERROR;

    // 3. Routing: Accel and Gyro Data Ready to INT1 pin
    config[0] = LSM6DS3_INT1_CTRL;
    config[1] = 0x03; // bit0: XL_DRDY, bit1: G_DRDY
    if (HAL_I2C_Master_Transmit(hi2c, LSM6DS3_ADDR, config, 2, 10) != HAL_OK) return HAL_ERROR;

    // CTRL3_C (0x12) is left at its power-on-reset default (push-pull, active-high INT),
    // which already matches what INT1 needs here, so no extra write is required.

    return HAL_OK;
}

/**
 * Call this in your EXTI Callback triggered by the INT1 pin
 * Uses DMA to read 12 bytes (6 for Gyro, 6 for Accel) starting from 0x22
 */
HAL_StatusTypeDef LSM6DS3_ReadRaw_DMA(I2C_HandleTypeDef *hi2c, uint8_t *buffer) {
    // We read from OUTX_L_G (0x22) all the way through OUTZ_H_XL (0x2D)
    return HAL_I2C_Mem_Read_DMA(hi2c, LSM6DS3_ADDR, LSM6DS3_OUTX_L_G, 1, buffer, 12);
}

void LSM6DS3_ProcessData(uint8_t *raw, LSM6DS3_Data_t *data) {
    // 1. Reconstruct raw 16-bit signed integers (Little Endian)
    data->gx = (int16_t)((raw[1] << 8) | raw[0]);
    data->gy = (int16_t)((raw[3] << 8) | raw[2]);
    data->gz = (int16_t)((raw[5] << 8) | raw[4]);

    data->ax = (int16_t)((raw[7] << 8) | raw[6]);
    data->ay = (int16_t)((raw[9] << 8) | raw[8]);
    data->az = (int16_t)((raw[11] << 8) | raw[10]);

    // 2. Convert to milli-G (Fixed point)
    // Result = (raw * 61) / 1000
    data->accel_mg[0] = ((int32_t)data->ax * ACCEL_SENSITIVITY_2G) / 1000;
    data->accel_mg[1] = ((int32_t)data->ay * ACCEL_SENSITIVITY_2G) / 1000;
    data->accel_mg[2] = ((int32_t)data->az * ACCEL_SENSITIVITY_2G) / 1000;

    // 3. Convert to dps * 1000 (Fixed point)
    // Result = (raw * 8750) / 1000 -> degrees per second
    data->gyro_dps[0] = ((int32_t)data->gx * GYRO_SENSITIVITY_245) / 1000;
    data->gyro_dps[1] = ((int32_t)data->gy * GYRO_SENSITIVITY_245) / 1000;
    data->gyro_dps[2] = ((int32_t)data->gz * GYRO_SENSITIVITY_245) / 1000;
}

HAL_StatusTypeDef LSM6DS3_ReadRaw_Poll(I2C_HandleTypeDef *hi2c, uint8_t *buffer) {
    return HAL_I2C_Mem_Read(hi2c, LSM6DS3_ADDR, LSM6DS3_OUTX_L_G, 1, buffer, 12, 100);
}

HAL_StatusTypeDef LSM6DS3_PowerDown(I2C_HandleTypeDef *hi2c)
{
    uint8_t pd = 0x00;
    HAL_StatusTypeDef s1 = HAL_I2C_Mem_Write(hi2c, LSM6DS3_ADDR, LSM6DS3_CTRL1_XL,
                                              I2C_MEMADD_SIZE_8BIT, &pd, 1, 10);
    HAL_StatusTypeDef s2 = HAL_I2C_Mem_Write(hi2c, LSM6DS3_ADDR, LSM6DS3_CTRL2_G,
                                              I2C_MEMADD_SIZE_8BIT, &pd, 1, 10);
    return (s1 == HAL_OK && s2 == HAL_OK) ? HAL_OK : HAL_ERROR;
}

uint8_t LSM6DS3_IsPoweredDown(I2C_HandleTypeDef *hi2c) {
    uint8_t ctrl1_xl = 0xFF, ctrl2_g = 0xFF;
    if (HAL_I2C_Mem_Read(hi2c, LSM6DS3_ADDR, LSM6DS3_CTRL1_XL, 1, &ctrl1_xl, 1, 10) != HAL_OK) return 0;
    if (HAL_I2C_Mem_Read(hi2c, LSM6DS3_ADDR, LSM6DS3_CTRL2_G, 1, &ctrl2_g, 1, 10) != HAL_OK) return 0;
    return ((ctrl1_xl & 0xF0) == 0x00) && ((ctrl2_g & 0xF0) == 0x00); // ODR bits = 0000 on both -> power-down
}
