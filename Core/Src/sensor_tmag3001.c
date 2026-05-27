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

// DWT microsecond timer
static inline void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t dwt_us(void) {
    return DWT->CYCCNT / (SystemCoreClock / 1000000U);
}

static void tmag_delay_us(uint32_t us)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < (us * cycles_per_us)) {}
}

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

static void tmag_i2c_recover(void)
{
    I2C3_BusRecover_Fast();
}

// Send N SCL pulses with TCA channel already selected so the sensor sees them.
// A TMAG3001 read transaction is 7 bytes = 63 bits (56 data + 7 ACK).
// We send 72 pulses (8 bytes × 9 bits) to guarantee clocking out any stuck
// transaction, followed by a STOP condition to reset the sensor state machine.
// Uses 5us half-cycle (~100kHz) for ~720us total overhead per channel.
#define TMAG_RECOVERY_SCL_PULSES  72

static void tmag_send_scl_pulses_and_stop(int n_pulses)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    #define _DUS(us) do { uint32_t _s=DWT->CYCCNT; while((DWT->CYCCNT-_s)<(us)*cycles_per_us){} } while(0)

    CLEAR_BIT(hi2c3.Instance->CR1, I2C_CR1_PE);
    _DUS(5);
    while (READ_BIT(hi2c3.Instance->CR1, I2C_CR1_PE)) {}

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_8; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_9; HAL_GPIO_Init(GPIOC, &g);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    _DUS(5);

    // N SCL pulses at ~100kHz — sensor sees them because TCA channel is selected
    for (int i = 0; i < n_pulses; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); _DUS(5);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   _DUS(5);
    }
    // STOP: SDA low→high while SCL high
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); _DUS(5);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   _DUS(5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C3;
    g.Pin = GPIO_PIN_8; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_9; HAL_GPIO_Init(GPIOC, &g);

    SET_BIT(hi2c3.Instance->CR1, I2C_CR1_PE);
    _DUS(20);

    if (__HAL_I2C_GET_FLAG(&hi2c3, I2C_FLAG_BUSY)) {
        SET_BIT(hi2c3.Instance->CR1, I2C_CR1_ANFOFF);
        _DUS(10);
        CLEAR_BIT(hi2c3.Instance->CR1, I2C_CR1_PE);
        _DUS(10);
        while (READ_BIT(hi2c3.Instance->CR1, I2C_CR1_PE)) {}
        CLEAR_BIT(hi2c3.Instance->CR1, I2C_CR1_ANFOFF);
        SET_BIT(hi2c3.Instance->CR1, I2C_CR1_PE);
        _DUS(20);
    }

    #undef _DUS
}

// Pre-read recovery: select TCA channel, then send 72 SCL pulses to clock out
// any stuck transaction from the previous cycle.
static void tmag_recover_sensor_on_channel(uint8_t tca_ch_mask)
{
    tmag_send_scl_pulses_and_stop(TMAG_RECOVERY_SCL_PULSES);
    TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, tca_ch_mask);
}

// Post-read cleanup: send 72 SCL pulses while TCA channel is still selected,
// so sensors are left in a clean state before TCA deselect.
static void tmag_cleanup_channel(uint8_t tca_ch_mask)
{
    (void)tca_ch_mask;  // channel already selected by caller
    tmag_send_scl_pulses_and_stop(TMAG_RECOVERY_SCL_PULSES);
}

// Fast BUSY clear: STM32H7 errata — analog filter can set false BUSY after STOP.
// Also sends 9 SCL pulses to release any sensor holding SDA low.
// ~200us total. Only call when BUSY is actually set.
static void tmag_clear_false_busy(void)
{
    uint32_t cycles_per_us = SystemCoreClock / 1000000U;
    #define _DUSB(us) do { uint32_t _s=DWT->CYCCNT; while((DWT->CYCCNT-_s)<(us)*cycles_per_us){} } while(0)

    CLEAR_BIT(hi2c3.Instance->CR1, I2C_CR1_PE);
    _DUSB(5);
    while (READ_BIT(hi2c3.Instance->CR1, I2C_CR1_PE)) {}

    GPIO_InitTypeDef g = {0};
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    g.Pin = GPIO_PIN_8; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_9; HAL_GPIO_Init(GPIOC, &g);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
    _DUSB(5);

    // 9 SCL pulses to release any sensor holding SDA low
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); _DUSB(5);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);   _DUSB(5);
    }
    // STOP: SDA low→high while SCL high
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET); _DUSB(5);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);   _DUSB(5);

    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_8);
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_9);
    g.Mode = GPIO_MODE_AF_OD;
    g.Pull = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF4_I2C3;
    g.Pin = GPIO_PIN_8; HAL_GPIO_Init(GPIOA, &g);
    g.Pin = GPIO_PIN_9; HAL_GPIO_Init(GPIOC, &g);

    SET_BIT(hi2c3.Instance->CR1, I2C_CR1_PE);
    _DUSB(20);

    // ANFOFF errata fix if still BUSY
    for (int _a = 0; _a < 3; _a++) {
        if (!__HAL_I2C_GET_FLAG(&hi2c3, I2C_FLAG_BUSY)) break;
        SET_BIT(hi2c3.Instance->CR1, I2C_CR1_ANFOFF);
        _DUSB(5);
        CLEAR_BIT(hi2c3.Instance->CR1, I2C_CR1_PE);
        _DUSB(5);
        while (READ_BIT(hi2c3.Instance->CR1, I2C_CR1_PE)) {}
        CLEAR_BIT(hi2c3.Instance->CR1, I2C_CR1_ANFOFF);
        SET_BIT(hi2c3.Instance->CR1, I2C_CR1_PE);
        _DUSB(20);
    }

    #undef _DUSB
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

    // Pre-TCA: recover main bus if stuck (fixes TCA itself being stuck)
    if (__HAL_I2C_GET_FLAG(&hi2c3, I2C_FLAG_BUSY)) {
        tmag_i2c_recover();
    }

    if (TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, tca_ch_mask) != HAL_OK) {
        tmag_i2c_recover();
        return 0;
    }

    // Channel-level recovery: send 72 SCL pulses after TCA select to clock out
    // any stuck sensor state from the previous cycle. This is the primary recovery
    // for the first sensor on each channel.
    tmag_recover_sensor_on_channel(tca_ch_mask);

    for (int i = 0; i < TMAG3001_TOTAL_NUM; i++) {
        TMAG3001_Instance_t *inst = &g_tmag_list[i];
        if (!inst->inited) continue;
        if (inst->tca_ch_mask != tca_ch_mask) continue;

        // Per-sensor recovery: send SCL pulses + STOP + TCA reselect before each read.
        // 0x36 (A2=SDA) needs more pulses — its address pin is on SDA so it's more
        // likely to be mid-transaction after a TCA switch. Use 36 pulses for 0x36,
        // 18 pulses for other addresses.
        {
            int n_pulses = (inst->addr7 == TMAG3001_ADDR_A2_SDA) ? 36 : 18;
            tmag_send_scl_pulses_and_stop(n_pulses);
        }
        TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, tca_ch_mask);

        tmag3001_data_t data;
        HAL_StatusTypeDef read_status = HAL_ERROR;
        for (int retry = 0; retry < 2; retry++) {
            read_status = TMAG3001_ReadData(&inst->dev, &data);
            if (read_status == HAL_OK) break;
            // Full bus recovery + TCA reselect
            tmag_i2c_recover();
            TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, tca_ch_mask);
            // Channel-aware recovery if sensor still stuck after TCA reselect
            if (__HAL_I2C_GET_FLAG(&hi2c3, I2C_FLAG_BUSY)) {
                tmag_recover_sensor_on_channel(tca_ch_mask);
            }
        }

        if (read_status != HAL_OK) continue;

        if (!tmag_append_line(out, out_size, off, tca_ch_mask, inst->addr7, &data)) {
            TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, 0);
            return 0;
        }

        tmag_delay_us(150);
    }

    // Post-read cleanup removed: per-sensor recovery at the start of each read
    // handles bus cleanup, so no cleanup needed after the last sensor.

    TCA9548_Select(&hi2c3, TMAG3001_TCA_ADDR_7B, 0);
    return 1;
}

// TMAG硬件复位：通过I2C3_RESET线复位TCA Mux和所有TMAG传感器
static void tmag_hardware_reset(void)
{
    // Force-reset I2C3 peripheral before DeInit to avoid hanging on stuck bus
    __HAL_RCC_I2C3_FORCE_RESET();
    HAL_Delay(5);
    __HAL_RCC_I2C3_RELEASE_RESET();
    HAL_Delay(2);
    HAL_I2C_DeInit(&hi2c3);

    HAL_GPIO_WritePin(I2C3_RESET_GPIO_Port, I2C3_RESET_Pin, GPIO_PIN_RESET);
    HAL_Delay(300);

    HAL_GPIO_WritePin(I2C3_RESET_GPIO_Port, I2C3_RESET_Pin, GPIO_PIN_SET);
    HAL_Delay(300);

    extern void MX_I2C3_Init(void);
    MX_I2C3_Init();

    HAL_Delay(100);
}

void Sensor_TMAG3001_Init_All(void)
{
    int idx = 0;

    memset(g_tmag_list, 0, sizeof(g_tmag_list));
    dwt_init();

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
