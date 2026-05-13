#pragma once
#include "stm32h7xx_hal.h"

// addr7: 7-bit 地址 (0x70~0x77，依 A2..A0)
// chanMask: 置位 n 以选通通道 n（bit0→CH0 ... bit7→CH7）
static inline HAL_StatusTypeDef TCA9548_Select(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t chanMask)
{
    uint8_t b = chanMask;
    // Use 100ms timeout for TCA selection - the original 10ms was too tight
    // and could fail if bus has any contention
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(hi2c, addr7 << 1, &b, 1, 100);
    if (status != HAL_OK) {
        // Retry once after a small delay
        HAL_Delay(1);
        status = HAL_I2C_Master_Transmit(hi2c, addr7 << 1, &b, 1, 100);
    }
    return status;
}
