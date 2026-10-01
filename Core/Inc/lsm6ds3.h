/*
 * lsm6ds3.h — LSM6DS3 IMU driver interface
 *
 * Author: Luca Obwegs
 */
#ifndef INC_LSM6DS3_H_
#define INC_LSM6DS3_H_

#include "stm32l0xx_hal.h"

#define LSM6DS3_ADDR        (0x6A << 1) // Default (SDO to GND)

// Registers
#define LSM6DS3_CTRL1_XL    0x10
#define LSM6DS3_CTRL2_G     0x11
#define LSM6DS3_INT1_CTRL   0x0D
#define LSM6DS3_OUTX_L_G    0x22
#define LSM6DS3_OUTX_L_XL   0x28

typedef struct {
    int16_t ax, ay, az; // Raw values
    int16_t gx, gy, gz;

    // Processed Fixed-Point (scaled by 1000)
    int32_t accel_mg[3]; // milli-G
    int32_t gyro_dps[3]; // degrees per second * 1000
} LSM6DS3_Data_t;

HAL_StatusTypeDef LSM6DS3_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef LSM6DS3_ReadRaw_DMA(I2C_HandleTypeDef *hi2c, uint8_t *buffer);
void LSM6DS3_ProcessData(uint8_t *raw_buffer, LSM6DS3_Data_t *data);
HAL_StatusTypeDef LSM6DS3_ReadRaw_Poll(I2C_HandleTypeDef *hi2c, uint8_t *buffer);
HAL_StatusTypeDef LSM6DS3_PowerDown(I2C_HandleTypeDef *hi2c);
uint8_t LSM6DS3_IsPoweredDown(I2C_HandleTypeDef *hi2c);

#endif /* INC_LSM6DS3_H_ */
