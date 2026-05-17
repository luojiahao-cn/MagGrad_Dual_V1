#pragma once
#include "stm32h7xx_hal.h"

// TCA9548A I2C address
#define TCA9548A_ADDR_7B  0x70

// Select TCA9548A channel - no retry
// addr7: 7-bit TCA9548A address (0x70)
// chanMask: bit n selects channel n (bit0→CH0 ... bit7→CH7), 0 to deselect
static inline HAL_StatusTypeDef TCA9548_Select(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t chanMask)
{
    uint8_t b = chanMask;
    return HAL_I2C_Master_Transmit(hi2c, addr7 << 1, &b, 1, 100);
}

// Check if TCA9548A is present on I2C bus
// Returns HAL_OK if device responds
static inline HAL_StatusTypeDef TCA9548_Ping(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    return HAL_I2C_IsDeviceReady(hi2c, addr7 << 1, 3, 100);
}

// Initialize TCA9548A - verify device exists
static inline HAL_StatusTypeDef TCA9548_Init(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    if (TCA9548_Ping(hi2c, addr7) != HAL_OK)
        return HAL_ERROR;
    return TCA9548_Select(hi2c, addr7, 0);
}
