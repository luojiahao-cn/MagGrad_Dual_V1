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

static void tmag_hardware_reset(void);

// TMAG hardware reset via I2C3_RESET line
static void tmag_hardware_reset(void)
{
    HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);

    HAL_I2C_DeInit(&hi2c3);

    HAL_GPIO_WritePin(I2C3_RESET_GPIO_Port, I2C3_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);

    HAL_GPIO_WritePin(I2C3_RESET_GPIO_Port, I2C3_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(100);

    extern void MX_I2C3_Init(void);
    MX_I2C3_Init();

    HAL_NVIC_SetPriority(I2C3_EV_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_SetPriority(I2C3_ER_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(I2C3_ER_IRQn);

    HAL_Delay(100);
}

void Sensor_TMAG3001_Init_All(void)
{
    int idx = 0;

    memset(g_tmag_list, 0, sizeof(g_tmag_list));

    tmag_hardware_reset();

    // Test TCA
    if (TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, 0) != HAL_OK) {
        return;
    }

    for (int ci = 0; ci < 4 && idx < TMAG3001_TOTAL_NUM; ci++) {
        uint8_t mask = 1 << (ci + 1);

        if ((TMAG3001_ACTIVE_TCA_MASK & mask) == 0U) continue;

        // Select TCA channel directly
        if (TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, mask) != HAL_OK) {
            continue;
        }
        HAL_Delay(20);

        for (uint8_t sub = 0; sub < TMAG3001_PER_CHANNEL && idx < TMAG3001_TOTAL_NUM; sub++) {
            TMAG3001_Instance_t *inst = &g_tmag_list[idx];
            uint8_t addr = g_tmag_addrs[sub];

            if (TMAG3001_Init(&inst->dev, &hi2c3, addr) == HAL_OK) {
                inst->tca_ch_mask = mask;
                inst->addr7 = addr;
                inst->i2c = &hi2c3;
                inst->inited = 1;
            }
            idx++;
            HAL_Delay(20);
        }
    }

    // Count first-pass results
    int inited = 0;
    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        if (g_tmag_list[i].inited) inited++;
    }

    // Reinit phase: re-init only the channels that worked
    if (inited > 0) {
        // Save (mask, addr) from first pass
        uint8_t saved_masks[TMAG3001_TOTAL_NUM] = {0};
        uint8_t saved_addrs[TMAG3001_TOTAL_NUM] = {0};
        int saved_count = 0;
        for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
            if (g_tmag_list[i].inited) {
                saved_masks[saved_count] = g_tmag_list[i].tca_ch_mask;
                saved_addrs[saved_count] = g_tmag_list[i].addr7;
                saved_count++;
            }
        }

        memset(g_tmag_list, 0, sizeof(g_tmag_list));
        tmag_hardware_reset();

        int reinit_ok = 0;

        for (int si = 0; si < saved_count && reinit_ok < saved_count; si++) {
            uint8_t mask = saved_masks[si];
            uint8_t addr = saved_addrs[si];

            TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, 0);
            if (TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, mask) != HAL_OK) {
                continue;
            }
            HAL_Delay(20);

            TMAG3001_Instance_t *inst = &g_tmag_list[reinit_ok];
            if (TMAG3001_Init(&inst->dev, &hi2c3, addr) == HAL_OK) {
                inst->tca_ch_mask = mask;
                inst->addr7 = addr;
                inst->i2c = &hi2c3;
                inst->inited = 1;
                reinit_ok++;
            } else {
                I2C3_BusRecover();
            }
        }
    }
}

int Sensor_TMAG3001_ReadToCSV(uint8_t tca_ch_mask, char *out_line, size_t out_size)
{
    if (out_size < 512) return 0;

    int sensor_count = 0;
    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        TMAG3001_Instance_t *inst = &g_tmag_list[i];
        if (inst->inited && inst->tca_ch_mask == tca_ch_mask) {
            sensor_count++;
        }
    }

    if (sensor_count == 0) return 0;

    if (TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, tca_ch_mask) != HAL_OK)
        return 0;
    HAL_Delay(10);

    size_t off = 0;

    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        TMAG3001_Instance_t *inst = &g_tmag_list[i];
        if (!inst->inited) continue;
        if (inst->tca_ch_mask != tca_ch_mask) continue;

        tmag3001_data_t data;
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
                              tca_ch_mask,
                              inst->addr7,
                              data.x, data.y, data.z,
                              (int)(data.status & 0x01),
                              (int)(data.status & 0x02));
        if (written <= 0) break;
        off += (size_t)written;
        if (off >= out_size) break;

        HAL_Delay(5);
    }

    TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, 0);
    return (int)off;
}

void Sensor_TMAG3001_ReadAll(void)
{
    char line[512];

    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        if (!g_tmag_list[i].inited) continue;

        uint8_t mask = g_tmag_list[i].tca_ch_mask;
        int already_read = 0;
        for (int j = 0; j < i; j++) {
            if (g_tmag_list[j].inited && g_tmag_list[j].tca_ch_mask == mask) {
                already_read = 1;
                break;
            }
        }
        if (already_read) continue;

        int n = Sensor_TMAG3001_ReadToCSV(mask, line, sizeof(line));
        if (n > 0) {
            USB_Send_String(line);
        }
    }
}
