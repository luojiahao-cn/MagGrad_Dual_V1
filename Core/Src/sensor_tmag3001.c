#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "sensor_tmag3001.h"
#include "tca9548.h"

extern I2C_HandleTypeDef hi2c3;

// USB CDC发送函数声明
extern void USB_Send_String(char *str);

typedef struct {
    tmag3001_t dev;
    uint8_t inited;
    uint8_t tca_ch_mask;
    uint8_t addr7;  // assigned I2C address
    I2C_HandleTypeDef *i2c;
} TMAG3001_Instance_t;

static TMAG3001_Instance_t g_tmag_list[TMAG3001_TOTAL_NUM];

// 3 addresses per channel: A2=GND, A2=3V3, A2=SDA
static const uint8_t g_tmag_addrs[TMAG3001_PER_CHANNEL] = {
    TMAG3001_ADDR_A2_GND,   // A2 = GND
    TMAG3001_ADDR_A2_SDA,   // A2 = SDA
    TMAG3001_ADDR_A2_3V3    // A2 = 3V3
};

static HAL_StatusTypeDef tmag_select_bus(I2C_HandleTypeDef *hi2c, uint8_t tca_ch_mask)
{
    HAL_StatusTypeDef status;
    int retries = 10;

    // Retry TCA selection several times for robustness
    while (retries-- > 0) {
        status = TCA9548_Select(hi2c, TMAG3001_TCA_ADDR_7B, tca_ch_mask);
        if (status == HAL_OK) {
            return HAL_OK;
        }
        HAL_Delay(50);  // Wait before retry
    }
    return HAL_ERROR;
}

static HAL_StatusTypeDef tmag_deselect_bus(void)
{
    return tmag_select_bus(&hi2c3, 0);
}

void Sensor_TMAG3001_Init_All(void)
{
    printf("TMAG: starting init (%d channels x %d sensors)\r\n",
           TMAG3001_CHANNELS, TMAG3001_PER_CHANNEL);

    // I2C3: channels 1..TMAG3001_CHANNELS, with separate addresses per channel.
    uint8_t idx = 0;

    for (uint8_t ch = 1; ch <= TMAG3001_CHANNELS && idx < TMAG3001_TOTAL_NUM; ch++) {
        uint8_t mask = 1 << ch;

        // Skip inactive channels
        if ((TMAG3001_ACTIVE_TCA_MASK & mask) == 0U) {
            printf("TMAG CH%d: inactive\r\n", ch);
            idx += TMAG3001_PER_CHANNEL;
            continue;
        }

        // Select TCA channel
        printf("TMAG CH%d: selecting TCA mask=0x%02X\r\n", ch, mask);
        HAL_Delay(100);  // Delay before TCA select
        if (tmag_select_bus(&hi2c3, mask) != HAL_OK) {
            printf("TMAG CH%d: TCA select fail\r\n", ch);
            idx += TMAG3001_PER_CHANNEL;
            tmag_deselect_bus();
            continue;
        }
        printf("TMAG CH%d: TCA selected OK\r\n", ch);
        HAL_Delay(200);  // Delay after TCA select

        for (uint8_t sub = 0; sub < TMAG3001_PER_CHANNEL && idx < TMAG3001_TOTAL_NUM; sub++) {
            TMAG3001_Instance_t *inst = &g_tmag_list[idx];
            inst->tca_ch_mask = mask;
            inst->addr7 = g_tmag_addrs[sub];
            inst->i2c = &hi2c3;
            inst->inited = 0;

            if (TMAG3001_Init(&inst->dev, &hi2c3, inst->addr7) == HAL_OK) {
                inst->inited = 1;
                printf("TMAG CH%d Addr0x%02X: OK\r\n", ch, inst->addr7);
            } else {
                printf("TMAG CH%d Addr0x%02X: init fail\r\n", ch, inst->addr7);
            }

            idx++;
        }

        tmag_deselect_bus();
    }
}

// Read all TMAG3001 on one TCA channel and format as CSV
int Sensor_TMAG3001_ReadToCSV(uint8_t tca_ch_mask, char *out_line, size_t out_size)
{
    if (out_size < 512) return 0;  // Need enough space for all sensors

    size_t off = 0;
    int written = 0;

    // Count initialized sensors on this channel
    int sensor_count = 0;
    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        TMAG3001_Instance_t *inst = &g_tmag_list[i];
        if (inst->inited && inst->tca_ch_mask == tca_ch_mask) {
            sensor_count++;
        }
    }

    if (sensor_count == 0) return 0;
    if (tmag_select_bus(&hi2c3, tca_ch_mask) != HAL_OK) return 0;
    HAL_Delay(5);  // Wait for sensors to be ready

    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        TMAG3001_Instance_t *inst = &g_tmag_list[i];
        if (!inst->inited) continue;
        if (inst->tca_ch_mask != tca_ch_mask) continue;

        tmag3001_data_t data;
        if (TMAG3001_ReadData(&inst->dev, &data) != HAL_OK) {
            continue;
        }

        written = snprintf(out_line + off, out_size - off, "TMAG,%d,0x%02X,%d,%d,%d,%d,%d\r\n",
                          inst->tca_ch_mask,
                          inst->addr7,
                          data.x, data.y, data.z,
                          (int)(data.status & 0x01),
                          (int)(data.status & 0x02));
        if (written <= 0) break;
        off += (size_t)written;
        if (off >= out_size) break;
    }

    tmag_deselect_bus();
    return (int)off;
}

// Read all initialized TMAG3001 sensors and send CSV output
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
