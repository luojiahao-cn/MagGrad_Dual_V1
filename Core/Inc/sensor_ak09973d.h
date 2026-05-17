#pragma once
#include "ak09973d.h"
#include "i2c.h"

#define AK09973D_STATIC_CONFIG \
    {1, 0x02, 0x10}, {1, 0x04, 0x10}, {1, 0x08, 0x10}, \
    {1, 0x10, 0x10}, {1, 0x20, 0x10}, {1, 0x40, 0x10}, \
    {2, 0x02, 0x10}, {2, 0x04, 0x10}, {2, 0x08, 0x10}, \
    {2, 0x10, 0x10}, {2, 0x20, 0x10}, {2, 0x40, 0x10}

#define AK09973D_TCA_ADDR_7B  0x70
#define AK09973D_COUNT         12
#define AK09973D_ACTIVE_CNTL2  AK09973D_MODE_10HZ

void Sensor_AK09973D_Init_All(void);
void Sensor_AK09973D_ReadAll(void);
void Sensor_AK09973D_Debug_I2C1(void);
void TCA_Test(void);
