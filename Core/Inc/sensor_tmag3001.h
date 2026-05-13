#pragma once
#include "tmag3001.h"
#include "i2c.h"

// TMAG3001 configuration
// I2C3: 4 channels x 3 sensors = 12 total (three sensors per channel)
#define TMAG3001_TOTAL_NUM      12     // 12 total sensors
#define TMAG3001_CHANNELS       4      // 4 TCA channels (1-4)
#define TMAG3001_PER_CHANNEL     3      // 3 sensors per channel
#define TMAG3001_TCA_ADDR_7B   0x70   // TCA9548A address

// Enable all 4 TCA channels for TMAG3001.
#define TMAG3001_ACTIVE_TCA_MASK (0x02 | 0x04 | 0x08 | 0x10)

// Base addresses: A2=GND, A2=SDA, A2=3V3
#define TMAG3001_ADDR_A2_GND   0x34
#define TMAG3001_ADDR_A2_SDA    0x36
#define TMAG3001_ADDR_A2_3V3   0x35

void Sensor_TMAG3001_Init_All(void);
int Sensor_TMAG3001_ReadToCSV(uint8_t tca_ch_mask, char *out_line, size_t out_size);
void Sensor_TMAG3001_ReadAll(void);
