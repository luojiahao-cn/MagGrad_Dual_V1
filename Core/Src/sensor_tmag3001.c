#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "sensor_tmag3001.h"
#include "tca9548.h"

extern I2C_HandleTypeDef hi2c3;

extern void USB_Send_String(char *str);

typedef struct {
    tmag3001_t dev;
    uint8_t inited;
    uint8_t tca_ch_mask;
    uint8_t addr7;
    I2C_HandleTypeDef *i2c;
} TMAG3001_Instance_t;

static TMAG3001_Instance_t g_tmag_list[TMAG3001_TOTAL_NUM];

static const uint8_t g_tmag_addrs[TMAG3001_PER_CHANNEL] = {
    TMAG3001_ADDR_A2_GND,
    TMAG3001_ADDR_A2_SDA,
    TMAG3001_ADDR_A2_3V3
};

static HAL_StatusTypeDef tmag_select_bus(uint8_t tca_ch_mask)
{
    // Retry TCA selection multiple times for stability
    for (int retry = 0; retry < 5; retry++) {
        if (TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, tca_ch_mask) == HAL_OK) {
            return HAL_OK;
        }
        HAL_Delay(5);
    }
    return HAL_ERROR;
}

static HAL_StatusTypeDef tmag_deselect_bus(void)
{
    return tmag_select_bus(0);
}

void Sensor_TMAG3001_Init_All(void)
{
    int inited_count = 0;

    HAL_Delay(300);

    uint8_t idx = 0;

    for (uint8_t ch = 1; ch <= TMAG3001_CHANNELS && idx < TMAG3001_TOTAL_NUM; ch++) {
        uint8_t mask = 1 << ch;

        if ((TMAG3001_ACTIVE_TCA_MASK & mask) == 0U) {
            idx += TMAG3001_PER_CHANNEL;
            continue;
        }

        HAL_Delay(100);
        if (tmag_select_bus(mask) != HAL_OK) {
            idx += TMAG3001_PER_CHANNEL;
            continue;
        }

        HAL_Delay(200);

        for (uint8_t sub = 0; sub < TMAG3001_PER_CHANNEL && idx < TMAG3001_TOTAL_NUM; sub++) {
            TMAG3001_Instance_t *inst = &g_tmag_list[idx];
            inst->tca_ch_mask = mask;
            inst->addr7 = g_tmag_addrs[sub];
            inst->i2c = &hi2c3;
            inst->inited = 0;

            if (TMAG3001_Init(&inst->dev, &hi2c3, inst->addr7) == HAL_OK) {
                inst->inited = 1;
                inited_count++;
            }
            idx++;
            HAL_Delay(50);
        }

        tmag_deselect_bus();
        HAL_Delay(100);
    }

    printf("TMAG: %d/%d OK\r\n", inited_count, TMAG3001_TOTAL_NUM);
}

int Sensor_TMAG3001_ReadToCSV(uint8_t tca_ch_mask, char *out_line, size_t out_size)
{
    if (out_size < 512) return 0;

    int sensor_count = 0;
    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        if (g_tmag_list[i].inited && g_tmag_list[i].tca_ch_mask == tca_ch_mask) {
            sensor_count++;
        }
    }

    if (sensor_count == 0) return 0;

    if (tmag_select_bus(tca_ch_mask) != HAL_OK) return 0;
    HAL_Delay(10);

    size_t off = 0;

    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        TMAG3001_Instance_t *inst = &g_tmag_list[i];
        if (!inst->inited) continue;
        if (inst->tca_ch_mask != tca_ch_mask) continue;

        tmag3001_data_t data;
        // Retry reading sensor data
        int read_ok = 0;
        for (int retry = 0; retry < 3; retry++) {
            if (TMAG3001_ReadData(&inst->dev, &data) == HAL_OK) {
                read_ok = 1;
                break;
            }
            HAL_Delay(5);
        }
        if (!read_ok) continue;

        int written = snprintf(out_line + off, out_size - off, "TMAG,%d,0x%02X,%d,%d,%d,%d,%d\r\n",
                              inst->tca_ch_mask,
                              inst->addr7,
                              data.x, data.y, data.z,
                              (int)(data.status & 0x01),
                              (int)(data.status & 0x02));
        if (written <= 0) break;
        off += (size_t)written;
        if (off >= out_size) break;

        HAL_Delay(5);
    }

    tmag_deselect_bus();
    return (int)off;
}

void Sensor_TMAG3001_ReadAll(void)
{
    char line[512];

    for (uint8_t ch = 1; ch <= TMAG3001_CHANNELS; ch++) {
        uint8_t mask = 1 << ch;
        if ((TMAG3001_ACTIVE_TCA_MASK & mask) == 0U) continue;

        int n = Sensor_TMAG3001_ReadToCSV(mask, line, sizeof(line));
        if (n > 0) {
            USB_Send_String(line);
        }
    }
}
