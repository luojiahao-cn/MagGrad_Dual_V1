#pragma once
#include "stm32h7xx_hal.h"

// addr7: 7-bit 地址 (0x70~0x77，依 A2..A0)
// chanMask: 置位 n 以选通通道 n（bit0→CH0 ... bit7→CH7）
static inline HAL_StatusTypeDef TCA9548_Select(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t chanMask)
{
    uint8_t b = chanMask;
    return HAL_I2C_Master_Transmit(hi2c, addr7 << 1, &b, 1, 10);
}
