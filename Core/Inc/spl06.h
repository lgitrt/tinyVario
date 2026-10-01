/*
 * spl06.h — SPL06 barometer driver interface
 *
 * Author: Luca Obwegs
 */
#ifndef INC_SPL06_H_
#define INC_SPL06_H_

#include "stm32l0xx_hal.h"

#define SPL06_I2C_ADDR (0x76 << 1)
#define SPL06_MEAS_CFG      0x08
// Scale factor for 8x oversampling is 7864320 (see SPL06 datasheet compensation table)
#define K_SCALE 7864320

typedef struct {
    int16_t c0, c1;
    int32_t c00, c10;
    int16_t c01, c11, c20, c21, c30;
    int32_t last_t_raw_sc; // Stored for pressure compensation
} SPL06_Calib_t;

HAL_StatusTypeDef SPL06_Init(I2C_HandleTypeDef *hi2c);
int32_t SPL06_GetTemperature(I2C_HandleTypeDef *hi2c); // Output: mC (1/1000 deg C)
int32_t SPL06_GetPressure(I2C_HandleTypeDef *hi2c);    // Output: Pa (Pascals)
uint8_t SPL06_DataReady(I2C_HandleTypeDef *hi2c, uint8_t mask);
HAL_StatusTypeDef SPL06_EnterStandby(I2C_HandleTypeDef *hi2c);
uint8_t SPL06_IsInStandby(I2C_HandleTypeDef *hi2c);

#endif /* INC_SPL06_H_ */
