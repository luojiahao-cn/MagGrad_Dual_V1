#pragma once
#include "tmag3001.h"
#include "i2c.h"
#include <stddef.h>

// TMAG3001 configuration
// I2C3: 4 channels x 3 sensors = 12 total
#define TMAG3001_TOTAL_NUM      12
#define TMAG3001_CHANNELS       4
#define TMAG3001_PER_CHANNEL     3
#define TMAG3001_TCA_ADDR_7B   0x70
#define TMAG3001_ACTIVE_TCA_MASK (0x02 | 0x04 | 0x08 | 0x10)  // CH1-CH4

// Base addresses in read/output order: A2=GND, A2=3V3, A2=SDA
#define TMAG3001_ADDR_A2_GND   0x34
#define TMAG3001_ADDR_A2_3V3   0x35
#define TMAG3001_ADDR_A2_SDA   0x36

#define TMAG3001_READ_TCA_SETTLE_MS   1
#define TMAG3001_READ_RETRY_DELAY_MS  5
#define TMAG3001_READ_POST_DELAY_MS   1

void Sensor_TMAG3001_Init_All(void);
int Sensor_TMAG3001_ReadToCSV(uint8_t tca_ch_mask, char *out_line, size_t out_size);
int Sensor_TMAG3001_ReadAllToCSV(char *out, size_t out_size);
void Sensor_TMAG3001_ReadAll(void);
int Sensor_TMAG3001_GetCount(void);
