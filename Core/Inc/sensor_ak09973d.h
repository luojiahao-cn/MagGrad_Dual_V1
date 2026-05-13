#pragma once
#include "ak09973d.h"
#include "i2c.h"

// AK09973D configuration
#define AK09973D_TOTAL_NUM     12     // 12 total sensors
#define AK09973D_TCA_ADDR_7B   0x70   // TCA9548A address

void Sensor_AK09973D_Init_All(void);
int Sensor_AK09973D_ReadToCSV(I2C_HandleTypeDef *hi2c, uint8_t tca_ch_mask, char *out_line, size_t out_size);
void Sensor_AK09973D_ReadAll(void);
