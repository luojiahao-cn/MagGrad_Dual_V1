/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "icm42670.h"
#include "sensor_ak09973d.h"
#include "sensor_tmag3001.h"
#include "binary_writer.h"
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include "usbd_cdc_if.h"
#include "usbd_cdc.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
icm42670_raw_t imu;
icm42670_t icm = {
    .hspi = &hspi1,
    .cs_port = SPI1_CS_GPIO_Port,
    .cs_pin = SPI1_CS_Pin};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifndef SENSOR_OUTPUT_ICM
#define SENSOR_OUTPUT_ICM   1
#endif
#ifndef SENSOR_OUTPUT_AK
#define SENSOR_OUTPUT_AK    1
#endif
#ifndef SENSOR_OUTPUT_TMAG
#define SENSOR_OUTPUT_TMAG  1
#endif
#ifndef SENSOR_OUTPUT_FORMAT
#define SENSOR_OUTPUT_FORMAT SENSOR_OUTPUT_FORMAT_BINARY
#endif
#ifndef AK_ARRAY_OUTPUT_HZ
#define AK_ARRAY_OUTPUT_HZ 480U
#endif
#define MAGGRAD_PROJECT_NAME "MagGrad"
#define MAGGRAD_BOARD_NAME "MagGrad_AK_TMAG_V1"
#define MAGGRAD_PROTOCOL_NAME "maggrad_binary_v1"
#define CONT_DEFAULT_HZ AK_ARRAY_OUTPUT_HZ
#define ICM_DEFAULT_HZ 480U
#define TRIG_AUTO_DEFAULT_HZ 100U
#define TRIG_AUTO_MIN_HZ 1U
#define TRIG_AUTO_MAX_HZ 500U
#define RUNTIME_RATE_MIN_HZ 1U
#define RUNTIME_RATE_MAX_HZ 1000U

typedef enum {
    RT_STRATEGY_IDLE = 0,
    RT_STRATEGY_CONT,
    RT_STRATEGY_TRIG,
    RT_STRATEGY_TRIG_AUTO
} runtime_strategy_t;

typedef enum {
    RT_PROFILE_AUTO = 0,
    RT_PROFILE_LOW_NOISE,
    RT_PROFILE_STABLE
} runtime_profile_t;

#define RT_SENSOR_AK   0x01U
#define RT_SENSOR_TMAG 0x02U
#define RT_SENSOR_ICM  0x04U
#define RT_SENSOR_MAG  (RT_SENSOR_AK | RT_SENSOR_TMAG)
#define RT_SENSOR_ALL  (RT_SENSOR_AK | RT_SENSOR_TMAG | RT_SENSOR_ICM)

typedef struct {
    runtime_strategy_t strategy;
    uint8_t sensor_mask;
    uint32_t trigger_hz;
    uint32_t next_trigger_cycle;
    uint32_t next_ak_cycle;
    uint32_t next_icm_cycle;
    uint32_t led_activity_until_ms;
    uint32_t led_error_until_ms;
    uint8_t manual_trigger_pending;
    uint8_t ak_initialized;
    uint8_t tmag_initialized;
    uint8_t icm_initialized;
    uint32_t target_hz;
    uint32_t icm_hz;
    runtime_profile_t requested_profile;
    runtime_profile_t active_profile;
    char profile_status[16];
    char profile_id[40];
    char fallback_from[16];
    char warning[40];
} runtime_state_t;

static runtime_state_t g_rt = {
    .strategy = RT_STRATEGY_IDLE,
    .sensor_mask = 0U,
    .trigger_hz = TRIG_AUTO_DEFAULT_HZ,
    .target_hz = CONT_DEFAULT_HZ,
    .icm_hz = ICM_DEFAULT_HZ,
    .requested_profile = RT_PROFILE_AUTO,
    .active_profile = RT_PROFILE_LOW_NOISE,
    .profile_status = "low_noise",
    .profile_id = "ak_sdr0_odr500",
    .fallback_from = "NONE",
    .warning = "NONE"
};

static uint32_t g_binary_seq = 0U;
static uint32_t g_ak_frames = 0U;
static uint32_t g_tmag_frames = 0U;
static uint32_t g_icm_frames = 0U;
static uint32_t g_skipped = 0U;
static uint32_t g_errors = 0U;
static uint32_t g_last_stats_ms = 0U;
static uint8_t g_ak_active_cntl2 = AK09973D_ACTIVE_CNTL2;
static uint8_t g_tmag_dev_cfg1 = TMAG3001_DEV_CFG1_DEFAULT;
static uint8_t g_tmag_dev_cfg2 = (uint8_t)(TMAG3001_DEV_CFG2_CONTINUOUS | TMAG3001_DEV_CFG2_LOW_NOISE);
static char g_tmag_profile_id[40] = "tmag_lp_ln1_avg8x";
static uint8_t g_tmag_profile_explicit = 0U;

#define LED_ACTIVE GPIO_PIN_RESET
#define LED_INACTIVE GPIO_PIN_SET
#define LED_ACTIVITY_MS 120U
#define LED_ERROR_MS 2000U
#define LED_SELF_TEST_MS 500U

// USB CDC发送字符串
extern USBD_HandleTypeDef hUsbDeviceFS;

static int USB_CDC_IsBusy(void)
{
    USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
    return (hcdc != NULL && hcdc->TxState != 0U);
}

static void USB_Send_Buffer(const uint8_t *buf, uint16_t len)
{
    uint16_t sent = 0;

    while (sent < len) {
        uint16_t chunk = len - sent;

        uint32_t start = HAL_GetTick();
        while (USB_CDC_IsBusy()) {
            if ((HAL_GetTick() - start) > 100U) {
                return;
            }
        }

        start = HAL_GetTick();
        while (CDC_Transmit_FS((uint8_t *)(buf + sent), chunk) == USBD_BUSY) {
            if ((HAL_GetTick() - start) > 100U) {
                return;
            }
        }

        start = HAL_GetTick();
        while (USB_CDC_IsBusy()) {
            if ((HAL_GetTick() - start) > 100U) {
                return;
            }
        }

        sent += chunk;
    }
}

void USB_Send_Raw(const uint8_t *buf, uint16_t len)
{
    USB_Send_Buffer(buf, len);
}

static void Main_DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static int Main_DWT_Due(uint32_t *next_cycle, uint32_t period_cycles)
{
    uint32_t now = DWT->CYCCNT;

    if (*next_cycle == 0U) {
        *next_cycle = now;
    }
    if ((int32_t)(now - *next_cycle) < 0) {
        return 0;
    }

    *next_cycle += period_cycles;
    if ((int32_t)(now - *next_cycle) >= 0) {
        *next_cycle = now + period_cycles;
    }
    return 1;
}

void USB_Send_String(char *str)
{
    size_t len = strlen(str);
    if (len > UINT16_MAX) {
        len = UINT16_MAX;
    }
    USB_Send_Buffer((const uint8_t *)str, (uint16_t)len);
}

static void USB_Send_PrintBuffer(char *ptr, int len)
{
    if (len <= 0) {
        return;
    }
    while (len > 0) {
        uint16_t chunk = (len > UINT16_MAX) ? UINT16_MAX : (uint16_t)len;
        USB_Send_Buffer((const uint8_t *)ptr, chunk);
        ptr += chunk;
        len -= chunk;
    }
}

int _write(int file, char *ptr, int len)
{
    (void)file;
    USB_Send_PrintBuffer(ptr, len);
    HAL_UART_Transmit(&huart1, (uint8_t *)ptr, len, 100);
    return len;
}

static const char *Runtime_StrategyName(runtime_strategy_t strategy)
{
    switch (strategy) {
    case RT_STRATEGY_IDLE:
        return "IDLE";
    case RT_STRATEGY_CONT:
        return "CONT";
    case RT_STRATEGY_TRIG:
        return "TRIG";
    case RT_STRATEGY_TRIG_AUTO:
        return "TRIG_AUTO";
    default:
        return "?";
    }
}

static const char *Runtime_SensorName(uint8_t mask)
{
    switch (mask) {
    case RT_SENSOR_AK:
        return "AK";
    case RT_SENSOR_TMAG:
        return "TMAG";
    case RT_SENSOR_ICM:
        return "ICM";
    case RT_SENSOR_AK | RT_SENSOR_TMAG:
        return "AK_TMAG";
    case RT_SENSOR_AK | RT_SENSOR_ICM:
        return "AK_ICM";
    case RT_SENSOR_TMAG | RT_SENSOR_ICM:
        return "TMAG_ICM";
    case RT_SENSOR_ALL:
        return "ALL";
    case 0U:
        return "NONE";
    default:
        return "CUSTOM";
    }
}

static const char *Runtime_ProfileName(runtime_profile_t profile)
{
    switch (profile) {
    case RT_PROFILE_AUTO:
        return "AUTO";
    case RT_PROFILE_LOW_NOISE:
        return "LOW_NOISE";
    case RT_PROFILE_STABLE:
        return "STABLE";
    default:
        return "?";
    }
}

static void Runtime_SendLine(const char *line)
{
    char out[512];
    int n = snprintf(out, sizeof(out), "%s\r\n", line);
    if (n > 0) {
        USB_Send_String(out);
    }
}

static void Runtime_SetLed(uint8_t red_on, uint8_t green_on, uint8_t blue_on)
{
    HAL_GPIO_WritePin(LEDR_GPIO_Port, LEDR_Pin, red_on ? LED_ACTIVE : LED_INACTIVE);
    HAL_GPIO_WritePin(LEDG_GPIO_Port, LEDG_Pin, green_on ? LED_ACTIVE : LED_INACTIVE);
    HAL_GPIO_WritePin(LEDB_GPIO_Port, LEDB_Pin, blue_on ? LED_ACTIVE : LED_INACTIVE);
}

static void Runtime_LedSelfTest(void)
{
    Runtime_SetLed(1U, 0U, 0U);
    HAL_Delay(LED_SELF_TEST_MS);
    Runtime_SetLed(0U, 1U, 0U);
    HAL_Delay(LED_SELF_TEST_MS);
    Runtime_SetLed(0U, 0U, 1U);
    HAL_Delay(LED_SELF_TEST_MS);
    Runtime_SetLed(0U, 0U, 0U);
}

static void Runtime_MarkActivity(void)
{
    g_rt.led_activity_until_ms = HAL_GetTick() + LED_ACTIVITY_MS;
}

static void Runtime_MarkError(void)
{
    g_rt.led_error_until_ms = HAL_GetTick() + LED_ERROR_MS;
}

static void Runtime_UpdateLed(void)
{
    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - g_rt.led_error_until_ms) < 0) {
        uint8_t on = ((now / 100U) & 1U) == 0U;
        Runtime_SetLed(on, 0U, 0U);
        return;
    }

    if ((int32_t)(now - g_rt.led_activity_until_ms) < 0) {
        Runtime_SetLed(0U, 1U, 0U);
        return;
    }

    switch (g_rt.strategy) {
    case RT_STRATEGY_IDLE:
        Runtime_SetLed(0U, 0U, ((now / 1000U) & 1U) == 0U);
        break;
    case RT_STRATEGY_CONT:
        Runtime_SetLed(0U, 1U, 0U);
        break;
    case RT_STRATEGY_TRIG:
        Runtime_SetLed(0U, 0U, 1U);
        break;
    case RT_STRATEGY_TRIG_AUTO:
        Runtime_SetLed(0U, 1U, 1U);
        break;
    default:
        Runtime_SetLed(1U, 0U, 0U);
        break;
    }
}

static void Runtime_Uppercase(char *s)
{
    while (*s != '\0') {
        if (*s >= 'a' && *s <= 'z') {
            *s = (char)(*s - ('a' - 'A'));
        }
        s++;
    }
}

static int Runtime_ParseSensors(const char *s, uint8_t *mask)
{
    if (strcmp(s, "AK") == 0) {
        *mask = RT_SENSOR_AK;
    } else if (strcmp(s, "TMAG") == 0) {
        *mask = RT_SENSOR_TMAG;
    } else if (strcmp(s, "ICM") == 0) {
        *mask = RT_SENSOR_ICM;
    } else if (strcmp(s, "AK_TMAG") == 0) {
        *mask = RT_SENSOR_AK | RT_SENSOR_TMAG;
    } else if (strcmp(s, "AK_ICM") == 0) {
        *mask = RT_SENSOR_AK | RT_SENSOR_ICM;
    } else if (strcmp(s, "TMAG_ICM") == 0) {
        *mask = RT_SENSOR_TMAG | RT_SENSOR_ICM;
    } else if (strcmp(s, "ALL") == 0) {
        *mask = RT_SENSOR_ALL;
    } else {
        return 0;
    }
    return 1;
}

static int Runtime_ParseHz(const char *s, uint32_t *hz)
{
    char *end = NULL;
    unsigned long v = strtoul(s, &end, 10);

    if (s == end || end == NULL || *end != '\0' ||
        v < RUNTIME_RATE_MIN_HZ || v > RUNTIME_RATE_MAX_HZ) {
        return 0;
    }

    *hz = (uint32_t)v;
    return 1;
}

static int Runtime_ParseProfile(const char *s, runtime_profile_t *profile)
{
    if (strcmp(s, "AUTO") == 0) {
        *profile = RT_PROFILE_AUTO;
    } else if (strcmp(s, "LOW_NOISE") == 0) {
        *profile = RT_PROFILE_LOW_NOISE;
    } else if (strcmp(s, "STABLE") == 0) {
        *profile = RT_PROFILE_STABLE;
    } else {
        return 0;
    }
    return 1;
}

static uint8_t Runtime_TMAGDevCfg1ForAvg(const char *avg)
{
    if (strcasecmp(avg, "avg1x") == 0) {
        return (uint8_t)((TMAG3001_DEV_CFG1_DEFAULT & (uint8_t)~0x1CU) | TMAG3001_DEV_CFG1_CONV_AVG_1X);
    }
    if (strcasecmp(avg, "avg2x") == 0) {
        return (uint8_t)((TMAG3001_DEV_CFG1_DEFAULT & (uint8_t)~0x1CU) | TMAG3001_DEV_CFG1_CONV_AVG_2X);
    }
    if (strcasecmp(avg, "avg4x") == 0) {
        return (uint8_t)((TMAG3001_DEV_CFG1_DEFAULT & (uint8_t)~0x1CU) | TMAG3001_DEV_CFG1_CONV_AVG_4X);
    }
    if (strcasecmp(avg, "avg8x") == 0) {
        return (uint8_t)((TMAG3001_DEV_CFG1_DEFAULT & (uint8_t)~0x1CU) | TMAG3001_DEV_CFG1_CONV_AVG_8X);
    }
    if (strcasecmp(avg, "avg16x") == 0) {
        return (uint8_t)((TMAG3001_DEV_CFG1_DEFAULT & (uint8_t)~0x1CU) | TMAG3001_DEV_CFG1_CONV_AVG_16X);
    }
    return (uint8_t)((TMAG3001_DEV_CFG1_DEFAULT & (uint8_t)~0x1CU) | TMAG3001_DEV_CFG1_CONV_AVG_32X);
}

static int Runtime_ParseTmagProfile(const char *s, runtime_profile_t *profile,
                                    uint8_t *dev_cfg1, uint8_t *dev_cfg2,
                                    const char **profile_id)
{
    const char *avg = NULL;
    int lp_ln = 0;

    if (strncasecmp(s, "tmag_lp_ln1_", 12) == 0) {
        avg = s + 12;
        lp_ln = 1;
        *profile = RT_PROFILE_LOW_NOISE;
    } else if (strncasecmp(s, "TMAG_LP_LN1_", 12) == 0) {
        avg = s + 12;
        lp_ln = 1;
        *profile = RT_PROFILE_LOW_NOISE;
    } else if (strncasecmp(s, "tmag_lc_", 8) == 0) {
        avg = s + 8;
        lp_ln = 0;
        *profile = RT_PROFILE_STABLE;
    } else if (strncasecmp(s, "TMAG_LC_", 8) == 0) {
        avg = s + 8;
        lp_ln = 0;
        *profile = RT_PROFILE_STABLE;
    } else {
        return 0;
    }

    if (strcasecmp(avg, "avg1x") != 0 && strcasecmp(avg, "avg2x") != 0 &&
        strcasecmp(avg, "avg4x") != 0 && strcasecmp(avg, "avg8x") != 0 &&
        strcasecmp(avg, "avg16x") != 0 && strcasecmp(avg, "avg32x") != 0) {
        return 0;
    }

    *dev_cfg1 = Runtime_TMAGDevCfg1ForAvg(avg);
    *dev_cfg2 = (uint8_t)(TMAG3001_DEV_CFG2_CONTINUOUS |
                          (lp_ln ? TMAG3001_DEV_CFG2_LOW_NOISE : TMAG3001_DEV_CFG2_LOW_CURRENT));
    *profile_id = s;
    return 1;
}

static void Runtime_SetTmagAutoProfile(uint32_t hz, runtime_profile_t profile)
{
    int lp_ln = (profile != RT_PROFILE_STABLE);

    (void)hz;

    g_tmag_dev_cfg1 = Runtime_TMAGDevCfg1ForAvg("avg32x");
    g_tmag_dev_cfg2 = (uint8_t)(TMAG3001_DEV_CFG2_CONTINUOUS |
                                (lp_ln ? TMAG3001_DEV_CFG2_LOW_NOISE : TMAG3001_DEV_CFG2_LOW_CURRENT));
    snprintf(g_tmag_profile_id, sizeof(g_tmag_profile_id),
             lp_ln ? "tmag_lp_ln1_avg32x" : "tmag_lc_avg32x");
}

static int Runtime_TmagProfileVerified(const char *profile_id, uint32_t hz)
{
    if (profile_id == NULL) {
        return 0;
    }

    if (strstr(profile_id, "AVG32X") != NULL || strstr(profile_id, "avg32x") != NULL) {
        return hz <= 280U;
    }
    if (strstr(profile_id, "AVG16X") != NULL || strstr(profile_id, "avg16x") != NULL) {
        return hz <= 200U;
    }
    if (strstr(profile_id, "AVG8X") != NULL || strstr(profile_id, "avg8x") != NULL ||
        strstr(profile_id, "AVG4X") != NULL || strstr(profile_id, "avg4x") != NULL ||
        strstr(profile_id, "AVG2X") != NULL || strstr(profile_id, "avg2x") != NULL ||
        strstr(profile_id, "AVG1X") != NULL || strstr(profile_id, "avg1x") != NULL) {
        return hz <= 200U;
    }

    return 0;
}

static void Runtime_ClearWarning(void)
{
    strncpy(g_rt.warning, "NONE", sizeof(g_rt.warning));
    strncpy(g_rt.fallback_from, "NONE", sizeof(g_rt.fallback_from));
    strncpy(g_rt.profile_status, "low_noise", sizeof(g_rt.profile_status));
}

static uint8_t Runtime_AKCntl2ForHz(uint32_t hz, const char **profile_id,
                                    const char **profile_status,
                                    const char **fallback_from,
                                    const char **warning)
{
    *profile_id = "ak_sdr0_odr500";
    *profile_status = "verified";
    *fallback_from = "NONE";
    *warning = "NONE";

    if (hz <= 200U) {
        return AK09973D_MODE_500HZ;
    }
    if (hz <= 500U) {
        *profile_status = "unverified";
        *warning = "RATE_UNVERIFIED";
        return AK09973D_MODE_500HZ;
    }

    *profile_status = "unverified";
    *fallback_from = "ak_sdr0_odr1000";
    *warning = "RATE_UNVERIFIED";
    return AK09973D_MODE_500HZ;
}

static void Runtime_SelectProfile(uint8_t sensor_mask, uint32_t hz)
{
    const char *profile_id = "low_noise_default";
    const char *profile_status = "unverified";
    const char *fallback_from = "NONE";
    const char *warning = "NONE";

    g_rt.active_profile = (g_rt.requested_profile == RT_PROFILE_STABLE)
        ? RT_PROFILE_STABLE
        : RT_PROFILE_LOW_NOISE;

    if ((sensor_mask & RT_SENSOR_AK) != 0U) {
        g_ak_active_cntl2 = Runtime_AKCntl2ForHz(hz, &profile_id, &profile_status,
                                                 &fallback_from, &warning);
    } else if ((sensor_mask & RT_SENSOR_TMAG) != 0U) {
        if (g_tmag_profile_explicit == 0U) {
            Runtime_SetTmagAutoProfile(hz, g_rt.requested_profile);
        }
        profile_id = g_tmag_profile_id;
        if (Runtime_TmagProfileVerified(profile_id, hz)) {
            profile_status = "verified";
            warning = "NONE";
        } else {
            profile_status = "unverified";
            warning = "RATE_UNVERIFIED";
        }
    }

    strncpy(g_rt.profile_id, profile_id, sizeof(g_rt.profile_id));
    strncpy(g_rt.profile_status, profile_status, sizeof(g_rt.profile_status));
    strncpy(g_rt.fallback_from, fallback_from, sizeof(g_rt.fallback_from));
    strncpy(g_rt.warning, warning, sizeof(g_rt.warning));
}

static int Runtime_EnsureSensors(uint8_t sensor_mask, runtime_strategy_t strategy)
{
    int ok = 1;

    if ((sensor_mask & RT_SENSOR_ICM) != 0U && g_rt.icm_initialized == 0U) {
#if SENSOR_OUTPUT_ICM
        if (ICM42670_Init(&icm) == HAL_OK) {
            g_rt.icm_initialized = 1U;
        } else {
            ok = 0;
        }
#else
        ok = 0;
#endif
    }

    if ((sensor_mask & RT_SENSOR_AK) != 0U && g_rt.ak_initialized == 0U) {
#if SENSOR_OUTPUT_AK
        Sensor_AK09973D_Init_All();
        int ak_count = Sensor_AK09973D_GetCount();
        if (ak_count > 0) {
            g_rt.ak_initialized = 1U;
            if (ak_count < AK09973D_COUNT) {
                snprintf(g_rt.warning, sizeof(g_rt.warning), "AK_INIT_PARTIAL");
            }
        } else {
            ok = 0;
        }
#else
        ok = 0;
#endif
    }

    if ((sensor_mask & RT_SENSOR_TMAG) != 0U && g_rt.tmag_initialized == 0U) {
#if SENSOR_OUTPUT_TMAG
        Sensor_TMAG3001_Init_All();
        if (Sensor_TMAG3001_GetCount() > 0) {
            g_rt.tmag_initialized = 1U;
        } else {
            ok = 0;
        }
#else
        ok = 0;
#endif
    }

#if SENSOR_OUTPUT_AK
    if (ok != 0 && (sensor_mask & RT_SENSOR_AK) != 0U && strategy == RT_STRATEGY_CONT) {
        if (Sensor_AK09973D_SetContinuousMode_AllWithCntl2(g_ak_active_cntl2) != HAL_OK) {
            ok = 0;
        }
    }
#endif

#if SENSOR_OUTPUT_TMAG
    if (ok != 0 && (sensor_mask & RT_SENSOR_TMAG) != 0U) {
        HAL_StatusTypeDef status;
        if (strategy == RT_STRATEGY_TRIG || strategy == RT_STRATEGY_TRIG_AUTO) {
            status = Sensor_TMAG3001_SetTriggerProfile_All(g_tmag_dev_cfg1, g_tmag_dev_cfg2);
            if (status != HAL_OK && g_rt.requested_profile != RT_PROFILE_STABLE) {
                uint8_t fallback_cfg2 = (uint8_t)((g_tmag_dev_cfg2 & (uint8_t)~TMAG3001_DEV_CFG2_LOW_NOISE) |
                                                  TMAG3001_DEV_CFG2_LOW_CURRENT);
                status = Sensor_TMAG3001_SetTriggerProfile_All(g_tmag_dev_cfg1, fallback_cfg2);
                if (status == HAL_OK) {
                    g_rt.active_profile = RT_PROFILE_STABLE;
                    strncpy(g_rt.profile_status, "fallback", sizeof(g_rt.profile_status));
                    strncpy(g_rt.fallback_from, g_tmag_profile_id, sizeof(g_rt.fallback_from));
                    strncpy(g_rt.profile_id, "tmag_lc_same_avg", sizeof(g_rt.profile_id));
                    strncpy(g_rt.warning, "TMAG_LP_LN_INIT_FAILED", sizeof(g_rt.warning));
                }
            }
        } else {
            status = Sensor_TMAG3001_SetContinuousProfile_All(g_tmag_dev_cfg1, g_tmag_dev_cfg2);
            if (status != HAL_OK && g_rt.requested_profile != RT_PROFILE_STABLE) {
                uint8_t fallback_cfg2 = (uint8_t)((g_tmag_dev_cfg2 & (uint8_t)~TMAG3001_DEV_CFG2_LOW_NOISE) |
                                                  TMAG3001_DEV_CFG2_LOW_CURRENT);
                status = Sensor_TMAG3001_SetContinuousProfile_All(g_tmag_dev_cfg1, fallback_cfg2);
                if (status == HAL_OK) {
                    g_rt.active_profile = RT_PROFILE_STABLE;
                    strncpy(g_rt.profile_status, "fallback", sizeof(g_rt.profile_status));
                    strncpy(g_rt.fallback_from, g_tmag_profile_id, sizeof(g_rt.fallback_from));
                    strncpy(g_rt.profile_id, "tmag_lc_same_avg", sizeof(g_rt.profile_id));
                    strncpy(g_rt.warning, "TMAG_LP_LN_INIT_FAILED", sizeof(g_rt.warning));
                }
            }
        }
        if (status != HAL_OK) {
            ok = 0;
        }
    }
#endif

    return ok;
}

static void Runtime_SendStatus(void)
{
    char line[384];
    snprintf(line, sizeof(line),
             "OK STATUS strategy=%s sensors=%s mask=0x%02X target_hz=%lu actual_hz=%lu icm_hz=%lu icm_actual_hz=%lu profile=%s profile_id=%s profile_status=%s fallback_from=%s warning=%s init=AK:%u,TMAG:%u,ICM:%u ak_count=%d ak_bitmap=0x%03X frames=AK:%lu,TMAG:%lu,ICM:%lu skipped=%lu errors=%lu",
             Runtime_StrategyName(g_rt.strategy),
             Runtime_SensorName(g_rt.sensor_mask),
             g_rt.sensor_mask,
             (unsigned long)g_rt.target_hz,
             (unsigned long)g_rt.target_hz,
             (unsigned long)g_rt.icm_hz,
             (unsigned long)g_rt.icm_hz,
             Runtime_ProfileName(g_rt.active_profile),
             g_rt.profile_id,
             g_rt.profile_status,
             g_rt.fallback_from,
             g_rt.warning,
             g_rt.ak_initialized,
             g_rt.tmag_initialized,
             g_rt.icm_initialized,
             Sensor_AK09973D_GetCount(),
             (unsigned int)Sensor_AK09973D_GetInitBitmap(),
             (unsigned long)g_ak_frames,
             (unsigned long)g_tmag_frames,
             (unsigned long)g_icm_frames,
             (unsigned long)g_skipped,
             (unsigned long)g_errors);
    Runtime_SendLine(line);
}

static void Runtime_SetIdle(void)
{
    g_rt.strategy = RT_STRATEGY_IDLE;
    g_rt.sensor_mask = 0U;
    g_rt.manual_trigger_pending = 0U;
    g_rt.next_trigger_cycle = 0U;
    g_rt.next_ak_cycle = 0U;
    g_rt.next_icm_cycle = 0U;
}

static void Runtime_SetMode(runtime_strategy_t strategy, uint8_t sensor_mask, uint32_t hz)
{
    g_rt.strategy = strategy;
    g_rt.sensor_mask = sensor_mask;
    g_rt.trigger_hz = hz;
    g_rt.target_hz = hz;
    g_rt.manual_trigger_pending = 0U;
    g_rt.next_trigger_cycle = 0U;
    g_rt.next_ak_cycle = 0U;
    g_rt.next_icm_cycle = 0U;
}

static void Runtime_HandleCommand(char *line)
{
    char *tok[5] = {0};
    int ntok = 0;

    Runtime_Uppercase(line);
    if (strcmp(line, "ERR_OVERFLOW") == 0) {
        Runtime_MarkError();
        Runtime_SendLine("ERR LINE_OVERFLOW");
        return;
    }

    for (char *p = strtok(line, " \t");
         p != NULL && ntok < (int)(sizeof(tok) / sizeof(tok[0]));
         p = strtok(NULL, " \t")) {
        tok[ntok++] = p;
    }

    if (ntok == 0) {
        return;
    }

    if (strcmp(tok[0], "STATUS") == 0) {
        Runtime_SendStatus();
        return;
    }

    if (strcmp(tok[0], "INFO") == 0) {
        Runtime_SendLine("OK INFO project=" MAGGRAD_PROJECT_NAME " board=" MAGGRAD_BOARD_NAME " protocol=" MAGGRAD_PROTOCOL_NAME " sensors=AK,TMAG,ICM");
        return;
    }

    if (strcmp(tok[0], "CAPS") == 0) {
        Runtime_SendLine("OK CAPS profiles=AUTO,LOW_NOISE,STABLE cont_hz=1..1000 trig_auto_hz=1..500 ak=odr500:1..200:verified,odr500:201..500:unverified,odr1000:501..1000:not-default tmag_auto=lp_ln1_avg32x:1..280,lp_ln1_avg32x:281..500:unverified tmag=lp_ln1_avg{1,2,4,8,16,32}x,lc_avg{1,2,4,8,16,32}x icm=low_noise");
        return;
    }

    if (strcmp(tok[0], "PROFILE") == 0) {
        runtime_profile_t profile;
        uint8_t tmag_dev_cfg1 = TMAG3001_DEV_CFG1_DEFAULT;
        uint8_t tmag_dev_cfg2 = (uint8_t)(TMAG3001_DEV_CFG2_CONTINUOUS | TMAG3001_DEV_CFG2_LOW_NOISE);
        const char *tmag_profile_id = NULL;
        const char *reply_profile_id = NULL;
        if (ntok < 2) {
            Runtime_MarkError();
            Runtime_SendLine("ERR BAD_PROFILE");
            return;
        }
        if (Runtime_ParseProfile(tok[1], &profile)) {
            if (profile == RT_PROFILE_STABLE) {
                tmag_dev_cfg2 = (uint8_t)(TMAG3001_DEV_CFG2_CONTINUOUS | TMAG3001_DEV_CFG2_LOW_CURRENT);
                tmag_profile_id = "tmag_lc_avg8x";
            } else {
                tmag_profile_id = "tmag_lp_ln1_avg8x";
            }
            reply_profile_id = tok[1];
            g_tmag_profile_explicit = 0U;
        } else if (!Runtime_ParseTmagProfile(tok[1], &profile, &tmag_dev_cfg1,
                                             &tmag_dev_cfg2, &tmag_profile_id)) {
            Runtime_MarkError();
            Runtime_SendLine("ERR BAD_PROFILE");
            return;
        } else {
            reply_profile_id = tmag_profile_id;
            g_tmag_profile_explicit = 1U;
        }
        g_rt.requested_profile = profile;
        g_tmag_dev_cfg1 = tmag_dev_cfg1;
        g_tmag_dev_cfg2 = tmag_dev_cfg2;
        strncpy(g_tmag_profile_id, tmag_profile_id, sizeof(g_tmag_profile_id));
        Runtime_ClearWarning();
        Runtime_SelectProfile(g_rt.sensor_mask, g_rt.target_hz);
        char out[96];
        snprintf(out, sizeof(out), "OK PROFILE %s", reply_profile_id);
        Runtime_SendLine(out);
        return;
    }

    if (strcmp(tok[0], "MODE") == 0) {
        runtime_strategy_t strategy;
        uint8_t sensor_mask = 0U;
        uint32_t hz = g_rt.trigger_hz;

        if (ntok >= 2 && strcmp(tok[1], "IDLE") == 0) {
            Runtime_SetIdle();
            Runtime_SendLine("OK MODE IDLE");
            return;
        }
        if (ntok < 3) {
            Runtime_MarkError();
            Runtime_SendLine("ERR BAD_MODE");
            return;
        }

        if (strcmp(tok[1], "CONT") == 0) {
            strategy = RT_STRATEGY_CONT;
        } else if (strcmp(tok[1], "TRIG") == 0) {
            strategy = RT_STRATEGY_TRIG;
        } else if (strcmp(tok[1], "TRIG_AUTO") == 0) {
            strategy = RT_STRATEGY_TRIG_AUTO;
            hz = TRIG_AUTO_DEFAULT_HZ;
        } else {
            Runtime_MarkError();
            Runtime_SendLine("ERR BAD_MODE");
            return;
        }

        if (!Runtime_ParseSensors(tok[2], &sensor_mask)) {
            Runtime_MarkError();
            Runtime_SendLine("ERR BAD_SENSOR");
            return;
        }

        if ((strategy == RT_STRATEGY_CONT || strategy == RT_STRATEGY_TRIG_AUTO) && ntok >= 4) {
            if (!Runtime_ParseHz(tok[3], &hz)) {
                Runtime_MarkError();
                Runtime_SendLine("ERR BAD_RATE");
                return;
            }
        }
        if (strategy == RT_STRATEGY_CONT && ntok < 4) {
            hz = g_rt.target_hz;
        }
        if (strategy == RT_STRATEGY_TRIG_AUTO && hz > TRIG_AUTO_MAX_HZ) {
            Runtime_MarkError();
            Runtime_SendLine("ERR BAD_RATE");
            return;
        }

        Runtime_SelectProfile(sensor_mask, hz);
        if (!Runtime_EnsureSensors(sensor_mask, strategy)) {
            Runtime_MarkError();
            Runtime_SendLine("ERR SENSOR_INIT");
            return;
        }

        Runtime_SetMode(strategy, sensor_mask, hz);
        char out[96];
        snprintf(out, sizeof(out), "OK MODE %s %s %lu",
                 Runtime_StrategyName(strategy),
                 Runtime_SensorName(sensor_mask),
                 (unsigned long)g_rt.trigger_hz);
        Runtime_SendLine(out);
        return;
    }

    if (strcmp(tok[0], "RATE") == 0) {
        uint32_t hz;
        if (ntok < 2 || !Runtime_ParseHz(tok[1], &hz)) {
            Runtime_MarkError();
            Runtime_SendLine("ERR BAD_RATE");
            return;
        }
        if (g_rt.strategy == RT_STRATEGY_TRIG) {
            Runtime_MarkError();
            Runtime_SendLine("ERR BAD_MODE");
            return;
        }
        if (g_rt.strategy == RT_STRATEGY_TRIG_AUTO && hz > TRIG_AUTO_MAX_HZ) {
            Runtime_MarkError();
            Runtime_SendLine("ERR BAD_RATE");
            return;
        }
        g_rt.target_hz = hz;
        g_rt.trigger_hz = hz;
        g_rt.next_trigger_cycle = 0U;
        g_rt.next_ak_cycle = 0U;
        Runtime_SelectProfile(g_rt.sensor_mask, hz);
#if SENSOR_OUTPUT_AK
        if (g_rt.strategy == RT_STRATEGY_CONT && (g_rt.sensor_mask & RT_SENSOR_AK) != 0U && g_rt.ak_initialized != 0U) {
            (void)Sensor_AK09973D_SetContinuousMode_AllWithCntl2(g_ak_active_cntl2);
        }
#endif
        char out[48];
        snprintf(out, sizeof(out), "OK RATE %lu", (unsigned long)hz);
        Runtime_SendLine(out);
        return;
    }

    if (strcmp(tok[0], "ICM_RATE") == 0) {
        uint32_t hz;
        if (ntok < 2 || !Runtime_ParseHz(tok[1], &hz)) {
            Runtime_MarkError();
            Runtime_SendLine("ERR BAD_RATE");
            return;
        }
        g_rt.icm_hz = hz;
        g_rt.next_icm_cycle = 0U;
        char out[56];
        snprintf(out, sizeof(out), "OK ICM_RATE %lu", (unsigned long)hz);
        Runtime_SendLine(out);
        return;
    }

    if (strcmp(tok[0], "TRIG") == 0) {
        if (g_rt.strategy != RT_STRATEGY_TRIG && g_rt.strategy != RT_STRATEGY_TRIG_AUTO) {
            Runtime_MarkError();
            Runtime_SendLine("ERR BAD_MODE");
            return;
        }
        if ((g_rt.sensor_mask & RT_SENSOR_MAG) == 0U) {
            Runtime_SendLine("OK NO_MAG_SENSOR");
            return;
        }
        g_rt.manual_trigger_pending = 1U;
        Runtime_SendLine("OK TRIG");
        return;
    }

    Runtime_SendLine("ERR UNKNOWN");
    Runtime_MarkError();
}

static void Runtime_ProcessCommands(void)
{
    char line[128];
    while (USB_CDC_ReadLine(line, sizeof(line)) != 0) {
        Runtime_HandleCommand(line);
    }
}

static void Runtime_ReadICM(uint8_t *binary_frame, size_t binary_frame_size)
{
#if SENSOR_OUTPUT_ICM
    int n = ICM42670_ReadToBinary(&icm, binary_frame, binary_frame_size,
                                  &g_binary_seq, &g_icm_frames, &g_skipped, &g_errors);
    if (n > 0) {
        USB_Send_Raw(binary_frame, (uint16_t)n);
    }
#else
    (void)binary_frame;
    (void)binary_frame_size;
#endif
}

static void Runtime_ReadAKArray(uint8_t *binary_frame, size_t binary_frame_size)
{
#if SENSOR_OUTPUT_AK
    int n = Sensor_AK09973D_ReadArrayToBinary(binary_frame, binary_frame_size,
                                              &g_binary_seq, &g_ak_frames,
                                              &g_skipped, &g_errors);
    if (n > 0) {
        USB_Send_Raw(binary_frame, (uint16_t)n);
    }
#else
    (void)binary_frame;
    (void)binary_frame_size;
#endif
}

static void Runtime_ReadTMAGArray(uint8_t *binary_frame, size_t binary_frame_size)
{
#if SENSOR_OUTPUT_TMAG
    int n = Sensor_TMAG3001_ReadArrayToBinary(binary_frame, binary_frame_size,
                                              &g_binary_seq, &g_tmag_frames,
                                              &g_skipped, &g_errors);
    if (n > 0) {
        USB_Send_Raw(binary_frame, (uint16_t)n);
    }
#else
    (void)binary_frame;
    (void)binary_frame_size;
#endif
}

static void Runtime_TriggerMagAndRead(uint8_t *binary_frame, size_t binary_frame_size)
{
    if ((g_rt.sensor_mask & RT_SENSOR_AK) != 0U) {
#if SENSOR_OUTPUT_AK
        if (Sensor_AK09973D_TriggerSingle_All() != HAL_OK) {
            g_errors++;
            Runtime_MarkError();
        }
        Runtime_ReadAKArray(binary_frame, binary_frame_size);
        if (g_rt.strategy == RT_STRATEGY_TRIG) {
            Runtime_MarkActivity();
        }
#endif
    }

    if ((g_rt.sensor_mask & RT_SENSOR_TMAG) != 0U) {
#if SENSOR_OUTPUT_TMAG
        if (Sensor_TMAG3001_TriggerSingle_All() != HAL_OK) {
            g_errors++;
            Runtime_MarkError();
        }
        Runtime_ReadTMAGArray(binary_frame, binary_frame_size);
        if (g_rt.strategy == RT_STRATEGY_TRIG) {
            Runtime_MarkActivity();
        }
#endif
    }
}

static void Runtime_SendStatsIfDue(uint8_t *binary_frame, size_t binary_frame_size)
{
    uint32_t now = HAL_GetTick();
    if (g_rt.strategy == RT_STRATEGY_IDLE || (now - g_last_stats_ms) < 1000U) {
        return;
    }

    size_t off = 0;
    if (BinaryWriter_AppendStats(binary_frame, binary_frame_size, &off,
                                 g_binary_seq++, now,
                                 g_ak_frames, g_tmag_frames, g_icm_frames,
                                 g_skipped, g_errors)) {
        USB_Send_Raw(binary_frame, (uint16_t)off);
    }
    g_last_stats_ms = now;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  // Immediate test - LED GPIO should work even before clock init
  // This confirms firmware is running
  // Note: Can't printf yet - clock not configured
  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  Main_DWT_Init();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USB_DEVICE_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

  Runtime_SetLed(0U, 0U, 0U);
  Runtime_LedSelfTest();

  HAL_Delay(100);
  HAL_Delay(2000);  // 等待 USB CDC 准备好

  Runtime_SetIdle();
  printf("=== MAG READY IDLE ===\r\n");
  Runtime_SendLine("OK BOOT IDLE");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	    static uint8_t binary_frame[4096];
#if SENSOR_OUTPUT_FORMAT == SENSOR_OUTPUT_FORMAT_BINARY
    Runtime_ProcessCommands();

    if (g_rt.strategy != RT_STRATEGY_IDLE &&
        (g_rt.sensor_mask & RT_SENSOR_ICM) != 0U &&
        Main_DWT_Due(&g_rt.next_icm_cycle, SystemCoreClock / g_rt.icm_hz)) {
        Runtime_ReadICM(binary_frame, sizeof(binary_frame));
    }

    if (g_rt.strategy == RT_STRATEGY_CONT) {
        if ((g_rt.sensor_mask & RT_SENSOR_AK) != 0U &&
            Main_DWT_Due(&g_rt.next_ak_cycle, SystemCoreClock / g_rt.target_hz)) {
            Runtime_ReadAKArray(binary_frame, sizeof(binary_frame));
        }
        if ((g_rt.sensor_mask & RT_SENSOR_TMAG) != 0U &&
            Main_DWT_Due(&g_rt.next_trigger_cycle, SystemCoreClock / g_rt.target_hz)) {
            Runtime_ReadTMAGArray(binary_frame, sizeof(binary_frame));
        }
    } else if (g_rt.strategy == RT_STRATEGY_TRIG) {
        if (g_rt.manual_trigger_pending != 0U) {
            g_rt.manual_trigger_pending = 0U;
            Runtime_TriggerMagAndRead(binary_frame, sizeof(binary_frame));
        }
    } else if (g_rt.strategy == RT_STRATEGY_TRIG_AUTO) {
        uint32_t period_cycles = SystemCoreClock / g_rt.trigger_hz;
        if (g_rt.manual_trigger_pending != 0U ||
            Main_DWT_Due(&g_rt.next_trigger_cycle, period_cycles)) {
            g_rt.manual_trigger_pending = 0U;
            Runtime_TriggerMagAndRead(binary_frame, sizeof(binary_frame));
        }
    }

    Runtime_SendStatsIfDue(binary_frame, sizeof(binary_frame));

#else
    Runtime_ProcessCommands();
#endif

    Runtime_UpdateLed();

    /* USER CODE END 3 */
  }
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 48;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
