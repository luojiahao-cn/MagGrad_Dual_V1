#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

#include "main.h"
#include "i2c.h"
#include "sensor_ak09973d.h"
#include "tca9548.h"

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c2;

static const struct {
    uint8_t i2c_bus;
    uint8_t mask;
    uint8_t addr;
} g_config[] = {
    AK09973D_STATIC_CONFIG
};

// 有效传感器列表
static struct {
    uint8_t i2c_bus;
    uint8_t mask;
    ak09973d_t dev;
} g_valid[AK09973D_COUNT];

static I2C_HandleTypeDef* get_i2c(uint8_t bus)
{
    return (bus == 1) ? &hi2c1 : &hi2c2;
}

static const char *hal_status_name(HAL_StatusTypeDef status)
{
    switch (status) {
    case HAL_OK:
        return "OK";
    case HAL_ERROR:
        return "ERROR";
    case HAL_BUSY:
        return "BUSY";
    case HAL_TIMEOUT:
        return "TIMEOUT";
    default:
        return "?";
    }
}

static void print_i2c_diag(const char *tag, I2C_HandleTypeDef *hi2c)
{
    uint32_t err = HAL_I2C_GetError(hi2c);
    HAL_I2C_StateTypeDef state = HAL_I2C_GetState(hi2c);
    GPIO_PinState scl = GPIO_PIN_SET;
    GPIO_PinState sda = GPIO_PIN_SET;

    if (hi2c == &hi2c1) {
        scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8);
        sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9);
    } else if (hi2c == &hi2c2) {
        scl = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10);
        sda = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
    }

    printf("%s state=0x%02X err=0x%08lX SCL=%d SDA=%d\r\n",
           tag,
           (unsigned int)state,
           (unsigned long)err,
           (int)scl,
           (int)sda);
}

static void i2c1_gpio_bus_clear(void)
{
    GPIO_InitTypeDef gpio = {0};

    HAL_I2C_DeInit(&hi2c1);

    gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8 | GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);

    for (int i = 0; i < 18 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        HAL_Delay(1);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        HAL_Delay(1);
    }

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    HAL_Delay(1);

    MX_I2C1_Init();
    HAL_Delay(5);
}

static void ak_bus_reset(uint8_t bus)
{
    I2C_HandleTypeDef *hi2c = get_i2c(bus);
    IRQn_Type ev_irq = (bus == 1) ? I2C1_EV_IRQn : I2C2_EV_IRQn;
    IRQn_Type er_irq = (bus == 1) ? I2C1_ER_IRQn : I2C2_ER_IRQn;
    GPIO_TypeDef *reset_port = (bus == 1) ? I2C1_RESET_GPIO_Port : I2C2_RESET_GPIO_Port;
    uint16_t reset_pin = (bus == 1) ? I2C1_RESET_Pin : I2C2_RESET_Pin;

    printf("[AK] Reset I2C%d...\r\n", bus);

    HAL_NVIC_DisableIRQ(ev_irq);
    HAL_NVIC_DisableIRQ(er_irq);
    HAL_I2C_DeInit(hi2c);

    HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(reset_port, reset_pin, GPIO_PIN_SET);
    HAL_Delay(100);

    if (bus == 1) {
        MX_I2C1_Init();
    } else {
        MX_I2C2_Init();
    }

    HAL_NVIC_SetPriority(ev_irq, 0, 0);
    HAL_NVIC_EnableIRQ(ev_irq);
    HAL_NVIC_SetPriority(er_irq, 0, 0);
    HAL_NVIC_EnableIRQ(er_irq);
    HAL_Delay(20);
    print_i2c_diag((bus == 1) ? "[AK] I2C1 after reset" : "[AK] I2C2 after reset", hi2c);
}

static HAL_StatusTypeDef ak_select_channel(uint8_t bus, uint8_t mask)
{
    I2C_HandleTypeDef *hi2c = get_i2c(bus);
    HAL_StatusTypeDef status = TCA9548_Select(hi2c, AK09973D_TCA_ADDR_7B, mask);

    if (status != HAL_OK) {
        if (bus == 1) {
            i2c1_gpio_bus_clear();
        }
        ak_bus_reset(bus);
        status = TCA9548_Select(hi2c, AK09973D_TCA_ADDR_7B, mask);
    }

    if (status == HAL_OK && bus == 1 && HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) {
        printf("[AK] I2C1 CH0x%02X holds SDA low, bus-clear retry\r\n", mask);
        i2c1_gpio_bus_clear();
        ak_bus_reset(1);
        status = TCA9548_Select(&hi2c1, AK09973D_TCA_ADDR_7B, mask);
    }

    return status;
}

// Test TCA on each channel - call from main for debugging
void TCA_Test(void)
{
    printf("[TCA] Test I2C1...\r\n");
    for (int ch = 1; ch <= 6; ch++) {
        uint8_t mask = 1 << (ch - 1);
        HAL_StatusTypeDef tca = TCA9548_Select(&hi2c1, AK09973D_TCA_ADDR_7B, mask);
        printf("  CH%d: %s\r\n", ch, tca == HAL_OK ? "OK" : "FAIL");
    }

    printf("[TCA] Test I2C2...\r\n");
    for (int ch = 1; ch <= 6; ch++) {
        uint8_t mask = 1 << (ch - 1);
        HAL_StatusTypeDef tca = TCA9548_Select(&hi2c2, AK09973D_TCA_ADDR_7B, mask);
        printf("  CH%d: %s\r\n", ch, tca == HAL_OK ? "OK" : "FAIL");
    }
}

void Sensor_AK09973D_Debug_I2C1(void)
{
    const uint8_t masks[] = {0x02, 0x04, 0x08, 0x10, 0x20, 0x40};

    printf("=== I2C1 ISOLATION DEBUG START ===\r\n");
    for (size_t i = 0; i < sizeof(masks); i++) {
        uint8_t mask = masks[i];
        uint8_t company = 0;
        uint8_t device = 0;

        ak_bus_reset(1);
        HAL_Delay(20);

        printf("[I2C1DBG] mask=0x%02X before ping: ", mask);
        print_i2c_diag("", &hi2c1);

        HAL_StatusTypeDef ping = TCA9548_Ping(&hi2c1, AK09973D_TCA_ADDR_7B);
        printf("[I2C1DBG] mask=0x%02X tca-ping=%s ", mask, hal_status_name(ping));
        print_i2c_diag("", &hi2c1);

        HAL_StatusTypeDef deselect = TCA9548_Select(&hi2c1, AK09973D_TCA_ADDR_7B, 0);
        printf("[I2C1DBG] mask=0x%02X deselect=%s ", mask, hal_status_name(deselect));
        print_i2c_diag("", &hi2c1);

        HAL_StatusTypeDef select = TCA9548_Select(&hi2c1, AK09973D_TCA_ADDR_7B, mask);
        printf("[I2C1DBG] mask=0x%02X select=%s ", mask, hal_status_name(select));
        print_i2c_diag("", &hi2c1);

        if (select == HAL_OK) {
            HAL_Delay(5);
            if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_RESET) {
                printf("[I2C1DBG] mask=0x%02X SDA low after select, bus-clear begin\r\n", mask);
                i2c1_gpio_bus_clear();
                print_i2c_diag("[I2C1DBG] after bus-clear", &hi2c1);
                select = TCA9548_Select(&hi2c1, AK09973D_TCA_ADDR_7B, mask);
                printf("[I2C1DBG] mask=0x%02X reselect=%s ", mask, hal_status_name(select));
                print_i2c_diag("", &hi2c1);
                HAL_Delay(5);
            }
            HAL_StatusTypeDef probe = AK09973D_Probe(&hi2c1, AK09973D_ADDR_1, &company, &device);
            printf("[I2C1DBG] mask=0x%02X wia=%s cid=0x%02X did=0x%02X ",
                   mask,
                   hal_status_name(probe),
                   company,
                   device);
            print_i2c_diag("", &hi2c1);

            HAL_StatusTypeDef off = TCA9548_Select(&hi2c1, AK09973D_TCA_ADDR_7B, 0);
            printf("[I2C1DBG] mask=0x%02X off-after-wia=%s ", mask, hal_status_name(off));
            print_i2c_diag("", &hi2c1);
        }
    }
    printf("=== I2C1 ISOLATION DEBUG END ===\r\n");
}

void Sensor_AK09973D_Init_All(void)
{
    memset(g_valid, 0, sizeof(g_valid));
    ak_bus_reset(1);
    ak_bus_reset(2);

    TCA9548_Select(&hi2c1, AK09973D_TCA_ADDR_7B, 0);
    TCA9548_Select(&hi2c2, AK09973D_TCA_ADDR_7B, 0);

    int count = 0;
    int tca_fail_bus1 = 0, tca_fail_bus2 = 0;
    int dev_fail_bus1 = 0, dev_fail_bus2 = 0;
    int init_bus1 = 0, init_bus2 = 0;

    printf("[AK] Init start cntl2=0x%02X (mode=0x%02X, low-noise, high-sens)\r\n",
           AK09973D_ACTIVE_CNTL2,
           AK09973D_ACTIVE_CNTL2 & AK09973D_CNTL2_MODE_MASK);
    for (int i = 0; i < AK09973D_COUNT; i++) {
        I2C_HandleTypeDef *hi2c = get_i2c(g_config[i].i2c_bus);
        uint8_t mask = g_config[i].mask;
        uint8_t addr = g_config[i].addr;
        uint8_t company = 0;
        uint8_t device = 0;
        ak09973d_config_t cfg;

        if (g_config[i].i2c_bus == 1) init_bus1++;
        else init_bus2++;

        printf("[AK] I2C%d CH0x%02X addr=0x%02X: ", g_config[i].i2c_bus, mask, addr);

        HAL_StatusTypeDef tca = ak_select_channel(g_config[i].i2c_bus, mask);
        if (tca != HAL_OK) {
            printf("TCA %s err=0x%08lX\r\n", hal_status_name(tca), (unsigned long)HAL_I2C_GetError(hi2c));
            if (g_config[i].i2c_bus == 1) tca_fail_bus1++;
            else tca_fail_bus2++;
            continue;
        }
        printf("TCA ok -> ");

        HAL_StatusTypeDef probe = AK09973D_Probe(hi2c, addr, &company, &device);
        if (probe != HAL_OK) {
            printf("WIA FAIL cid=0x%02X did=0x%02X err=0x%08lX\r\n",
                   company, device, (unsigned long)HAL_I2C_GetError(hi2c));
            if (g_config[i].i2c_bus == 1) dev_fail_bus1++;
            else dev_fail_bus2++;
            continue;
        }
        printf("WIA=0x%02X%02X -> ", company, device);

        ak09973d_t dev = {0};
        HAL_StatusTypeDef ak = AK09973D_InitWithConfig(&dev, hi2c, addr, AK09973D_ACTIVE_CNTL2);
        if (ak != HAL_OK) {
            printf("INIT %s err=0x%08lX\r\n", hal_status_name(ak), (unsigned long)HAL_I2C_GetError(hi2c));
            if (g_config[i].i2c_bus == 1) dev_fail_bus1++;
            else dev_fail_bus2++;
            continue;
        }
        if (AK09973D_ReadConfig(&dev, &cfg) == HAL_OK) {
            printf("OK cntl1=0x%02X%02X cntl2=0x%02X\r\n", cfg.cntl1_h, cfg.cntl1_l, cfg.cntl2);
        } else {
            printf("OK cfg-read-fail\r\n");
        }

        // 保存有效的传感器
        g_valid[count].i2c_bus = g_config[i].i2c_bus;
        g_valid[count].mask = mask;
        g_valid[count].dev = dev;
        count++;

        TCA9548_Select(hi2c, AK09973D_TCA_ADDR_7B, 0);
    }

    printf("[AK] Result: %d/%d (I2C1: init=%d tca=%d dev=%d, I2C2: init=%d tca=%d dev=%d)\r\n",
           count, AK09973D_COUNT,
           init_bus1, tca_fail_bus1, dev_fail_bus1,
           init_bus2, tca_fail_bus2, dev_fail_bus2);

    // LED反馈：成功数闪烁
    for (int i = 0; i < count; i++) {
        HAL_GPIO_TogglePin(LEDG_GPIO_Port, LEDG_Pin);
        HAL_Delay(30);
    }
    if (count == AK09973D_COUNT) {
        // 全部成功 - 绿灯亮一下
        HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_RESET);
        HAL_Delay(100);
        HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, GPIO_PIN_SET);
    } else if (count == 0) {
        // 全失败 - 红灯亮
        HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, GPIO_PIN_RESET);
    }
}

void Sensor_AK09973D_ReadAll(void)
{
    for (int i = 0; i < AK09973D_COUNT && g_valid[i].dev.hi2c != NULL; i++) {
        I2C_HandleTypeDef *hi2c = get_i2c(g_valid[i].i2c_bus);

        if (ak_select_channel(g_valid[i].i2c_bus, g_valid[i].mask) != HAL_OK) {
            printf("AKERR,%d,0x%02X,TCA,0x%08lX\r\n",
                   g_valid[i].i2c_bus, g_valid[i].mask, (unsigned long)HAL_I2C_GetError(hi2c));
            continue;
        }

        ak09973d_magdata_t data;
        if (AK09973D_ReadMagData(&g_valid[i].dev, &data) != HAL_OK) {
            printf("AKERR,%d,0x%02X,READ,0x%08lX\r\n",
                   g_valid[i].i2c_bus, g_valid[i].mask, (unsigned long)HAL_I2C_GetError(hi2c));
            continue;
        }

        printf("AK,%d,0x%02X,%d,%d,%d,%d,%d,%d\r\n",
               g_valid[i].i2c_bus,
               g_valid[i].mask,
               data.hx, data.hy, data.hz,
               (int)(data.status & 0x01),
               (int)((data.status & AK09973D_ST_ERR) != 0U),
               (int)((data.status & AK09973D_ST_DOR) != 0U));
    }

    TCA9548_Select(&hi2c1, AK09973D_TCA_ADDR_7B, 0);
    TCA9548_Select(&hi2c2, AK09973D_TCA_ADDR_7B, 0);
}
