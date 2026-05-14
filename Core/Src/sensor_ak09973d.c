#include "stm32h7xx_hal.h"
#include <stdio.h>

#include "main.h"
#include "i2c.h"
#include "sensor_ak09973d.h"
#include "tca9548.h"

extern void USB_Send_String(char *str);

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

#define TCA_ADDR_7B     0x70
#define AK09973D_ADDR_7B 0x10

typedef struct {
    ak09973d_t dev;
    uint8_t inited;
    I2C_HandleTypeDef *i2c;
    uint8_t tca_ch_mask;
} AK09973D_Instance_t;

static AK09973D_Instance_t g_ak_list[AK09973D_TOTAL_NUM];

static AK09973D_Instance_t* find_ak_instance(I2C_HandleTypeDef *hi2c, uint8_t tca_ch_mask)
{
    for (int i = 0; i < AK09973D_TOTAL_NUM; i++) {
        if (g_ak_list[i].inited &&
            g_ak_list[i].i2c == hi2c &&
            g_ak_list[i].tca_ch_mask == tca_ch_mask) {
            return &g_ak_list[i];
        }
    }
    return NULL;
}

static AK09973D_Instance_t* alloc_ak_slot(void)
{
    for (int i = 0; i < AK09973D_TOTAL_NUM; i++) {
        if (!g_ak_list[i].inited) {
            return &g_ak_list[i];
        }
    }
    return NULL;
}

void Sensor_AK09973D_Init_All(void)
{
    int i2c1_count = 0, i2c2_count = 0;

    // I2C1: sensors on channels 1-6
    for (uint8_t ch = 1; ch < 7; ch++) {
        uint8_t mask = 1 << ch;

        if (TCA9548_Select(&hi2c1, TCA_ADDR_7B, mask) != HAL_OK) continue;
        if (HAL_I2C_IsDeviceReady(&hi2c1, AK09973D_ADDR_7B << 1, 3, 100) != HAL_OK) continue;

        AK09973D_Instance_t *inst = alloc_ak_slot();
        if (inst == NULL) continue;

        inst->i2c = &hi2c1;
        inst->tca_ch_mask = mask;

        if (AK09973D_Init(&inst->dev, &hi2c1, AK09973D_ADDR_7B) == HAL_OK) {
            inst->inited = 1;
            i2c1_count++;
        }
    }

    // I2C2: sensors on channels 1-6
    for (uint8_t ch = 1; ch < 7; ch++) {
        uint8_t mask = 1 << ch;

        if (TCA9548_Select(&hi2c2, TCA_ADDR_7B, mask) != HAL_OK) continue;
        if (HAL_I2C_IsDeviceReady(&hi2c2, AK09973D_ADDR_7B << 1, 3, 100) != HAL_OK) continue;

        AK09973D_Instance_t *inst = alloc_ak_slot();
        if (inst == NULL) continue;

        inst->i2c = &hi2c2;
        inst->tca_ch_mask = mask;

        if (AK09973D_Init(&inst->dev, &hi2c2, AK09973D_ADDR_7B) == HAL_OK) {
            inst->inited = 1;
            i2c2_count++;
        }
    }

    printf("AK: I2C1=%d, I2C2=%d\r\n", i2c1_count, i2c2_count);
}

int Sensor_AK09973D_ReadToCSV(I2C_HandleTypeDef *hi2c, uint8_t tca_ch_mask, char *out_line, size_t out_size)
{
    if (out_size < 64) return 0;

    AK09973D_Instance_t *inst = find_ak_instance(hi2c, tca_ch_mask);
    if (inst == NULL || !inst->inited) return 0;

    if (TCA9548_Select(hi2c, TCA_ADDR_7B, tca_ch_mask) != HAL_OK) {
        TCA9548_Select(hi2c, TCA_ADDR_7B, 0);
        return 0;
    }

    ak09973d_magdata_t data;
    if (AK09973D_ReadMagData(&inst->dev, &data) != HAL_OK) {
        TCA9548_Select(hi2c, TCA_ADDR_7B, 0);
        return 0;
    }

    int n = snprintf(out_line, out_size, "AK,%d,%d,%d,%d,%d,%d,%d\r\n",
                     tca_ch_mask,
                     data.hx, data.hy, data.hz,
                     (int)(data.status & 0x01),
                     (int)(data.status & 0x20));

    TCA9548_Select(hi2c, TCA_ADDR_7B, 0);
    return (n > 0 && (size_t)n < out_size) ? n : 0;
}

void Sensor_AK09973D_ReadAll(void)
{
    char line[128];

    for (uint8_t ch = 1; ch < 7; ch++) {
        int n = Sensor_AK09973D_ReadToCSV(&hi2c1, 1 << ch, line, sizeof(line));
        if (n > 0) USB_Send_String(line);
    }

    for (uint8_t ch = 1; ch < 7; ch++) {
        int n = Sensor_AK09973D_ReadToCSV(&hi2c2, 1 << ch, line, sizeof(line));
        if (n > 0) USB_Send_String(line);
    }
}
