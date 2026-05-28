#pragma once
#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stddef.h>

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;
} icm42670_t;

typedef struct
{
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp; // 原始计数，°C = temp/128 + 25
} icm42670_raw_t;

HAL_StatusTypeDef ICM42670_Init(icm42670_t *icm);
HAL_StatusTypeDef ICM42670_ReadRaw(icm42670_t *icm, icm42670_raw_t *out);
uint8_t ICM42670_IsDataReady(icm42670_t *icm);
int ICM42670_ReadToCSV(icm42670_t *icm, char *out, size_t out_size);
int ICM42670_ReadToBinary(icm42670_t *icm, uint8_t *out, size_t out_size,
                          uint32_t *seq, uint32_t *frames,
                          uint32_t *skipped, uint32_t *errors);

void ICM42670_ReadAndPrint(icm42670_t *icm);
