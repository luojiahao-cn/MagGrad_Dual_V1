#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "csv_writer.h"
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
    TMAG3001_ADDR_A2_3V3,
    TMAG3001_ADDR_A2_SDA
};

static void tmag_hardware_reset(void);

static int tmag_append_line(char *out, size_t out_size, size_t *off,
                            uint8_t tca_ch_mask, uint8_t addr7,
                            const tmag3001_data_t *data)
{
    return CSV_AppendString(out, out_size, off, "TMAG,") &&
           CSV_AppendU32(out, out_size, off, tca_ch_mask) &&
           CSV_AppendChar(out, out_size, off, ',') &&
           CSV_AppendHex8(out, out_size, off, addr7) &&
           CSV_AppendChar(out, out_size, off, ',') &&
           CSV_AppendI32(out, out_size, off, data->x) &&
           CSV_AppendChar(out, out_size, off, ',') &&
           CSV_AppendI32(out, out_size, off, data->y) &&
           CSV_AppendChar(out, out_size, off, ',') &&
           CSV_AppendI32(out, out_size, off, data->z) &&
           CSV_AppendChar(out, out_size, off, ',') &&
           CSV_AppendU32(out, out_size, off, (data->status & 0x01U) ? 1U : 0U) &&
           CSV_AppendChar(out, out_size, off, ',') &&
           CSV_AppendU32(out, out_size, off, (data->status & 0x02U) ? 1U : 0U) &&
           CSV_AppendCRLF(out, out_size, off);
}

static int tmag_read_channel_to_csv(uint8_t tca_ch_mask, char *out, size_t out_size, size_t *off)
{
    int sensor_count = 0;
    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        TMAG3001_Instance_t *inst = &g_tmag_list[i];
        if (inst->inited && inst->tca_ch_mask == tca_ch_mask) {
            sensor_count++;
        }
    }

    if (sensor_count == 0) return 1;

    if (TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, tca_ch_mask) != HAL_OK) {
        return 0;
    }
#if TMAG3001_READ_TCA_SETTLE_MS > 0
    HAL_Delay(TMAG3001_READ_TCA_SETTLE_MS);
#endif

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
#if TMAG3001_READ_RETRY_DELAY_MS > 0
            HAL_Delay(TMAG3001_READ_RETRY_DELAY_MS);
#endif
        }
        if (!read_ok) continue;

        if (!tmag_append_line(out, out_size, off, tca_ch_mask, inst->addr7, &data)) {
            return 0;
        }

#if TMAG3001_READ_POST_DELAY_MS > 0
        HAL_Delay(TMAG3001_READ_POST_DELAY_MS);
#endif
    }

    TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, 0);
    return 1;
}

// TMAG硬件复位：通过I2C3_RESET线复位TCA Mux和所有TMAG传感器
static void tmag_hardware_reset(void)
{
    HAL_NVIC_DisableIRQ(I2C3_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C3_ER_IRQn);

    HAL_I2C_DeInit(&hi2c3);

    HAL_GPIO_WritePin(I2C3_RESET_GPIO_Port, I2C3_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(300);

    HAL_GPIO_WritePin(I2C3_RESET_GPIO_Port, I2C3_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(300);

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

    printf("TMAG: init start\r\n");
    fflush(stdout);

    tmag_hardware_reset();

    // Test TCA
    if (TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, 0) != HAL_OK) {
        printf("TMAG: TCA not responding - abort\r\n");
        fflush(stdout);
        return;
    }
    printf("TMAG: TCA OK\r\n");
    fflush(stdout);

    // Direct init: CH1, CH2, CH3, CH4 (normal order)
    uint8_t ch_order[] = {1, 2, 3, 4};
    for (int ci = 0; ci < 4 && idx < TMAG3001_TOTAL_NUM; ci++) {
        uint8_t ch = ch_order[ci];
        uint8_t mask = 1 << ch;

        if ((TMAG3001_ACTIVE_TCA_MASK & mask) == 0U) continue;

        int ch_start_idx = idx;
        int ch_ok = 0;
        for (int ch_attempt = 0; ch_attempt < 2 && ch_ok == 0; ch_attempt++) {
            idx = ch_start_idx;

        // Select TCA channel directly (no prior deselect needed)
        if (TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, mask) != HAL_OK) {
            printf("TMAG CH%d: TCA select FAIL\r\n", ch);
            fflush(stdout);
            tmag_hardware_reset();
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
                ch_ok++;
            } else {
                TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, 0);
                HAL_Delay(5);
                TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, mask);
                HAL_Delay(20);
            }
            idx++;
            HAL_Delay(20);
        }

        if (ch_ok == 0 && ch_attempt == 0) {
            TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, 0);
            HAL_Delay(10);
            tmag_hardware_reset();
        }
        }

        printf("TMAG CH%d: %d/%d OK\r\n", ch, ch_ok, TMAG3001_PER_CHANNEL);
        fflush(stdout);
        TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, 0);
        HAL_Delay(10);
    }

    // Count first-pass results
    int inited = 0;
    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        if (g_tmag_list[i].inited) inited++;
    }

    printf("TMAG: first pass %d/%d OK\r\n", inited, TMAG3001_TOTAL_NUM);
    fflush(stdout);

    printf("TMAG: init done\r\n");
    fflush(stdout);
}

int Sensor_TMAG3001_ReadToCSV(uint8_t tca_ch_mask, char *out_line, size_t out_size)
{
    size_t off = 0;
    (void)tmag_read_channel_to_csv(tca_ch_mask, out_line, out_size, &off);
    return (int)off;
}

void Sensor_TMAG3001_ReadAll(void)
{
    char frame[1024];
    int n = Sensor_TMAG3001_ReadAllToCSV(frame, sizeof(frame));
    if (n > 0) {
        USB_Send_String(frame);
    }
}

int Sensor_TMAG3001_ReadAllToCSV(char *out, size_t out_size)
{
    size_t off = 0;

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

        if (!tmag_read_channel_to_csv(mask, out, out_size, &off)) {
            break;
        }
    }

    return (int)off;
}

int Sensor_TMAG3001_GetCount(void)
{
    int count = 0;
    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        if (g_tmag_list[i].inited) {
            count++;
        }
    }
    return count;
}
