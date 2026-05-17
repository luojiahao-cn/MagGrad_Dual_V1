#ifndef __TMAG3001_H
#define __TMAG3001_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

// Default I2C Addresses (based on ADDR pin)
#define TMAG3001_ADDR_GND      0x34  // ADDR = GND
#define TMAG3001_ADDR_VCC      0x35  // ADDR = VCC
#define TMAG3001_ADDR_SDA     0x36  // ADDR = SDA
#define TMAG3001_ADDR_SCL      0x37  // ADDR = SCL

// Registers
#define TMAG3001_REG_DEV_CFG1     0x00
#define TMAG3001_REG_DEV_CFG2     0x01
#define TMAG3001_REG_SENS_CFG1    0x02
#define TMAG3001_REG_SENS_CFG2    0x03
#define TMAG3001_REG_THR_CFG1     0x04
#define TMAG3001_REG_THR_CFG2     0x05
#define TMAG3001_REG_THR_CFG3     0x06
#define TMAG3001_REG_SENS_CFG3    0x07
#define TMAG3001_REG_INT_CFG1     0x08
#define TMAG3001_REG_SENS_CFG4    0x09
#define TMAG3001_REG_SENS_CFG5    0x0A
#define TMAG3001_REG_SENS_CFG6    0x0B
#define TMAG3001_REG_I2C_ADDR     0x0C  // Programmable I2C address
#define TMAG3001_REG_DEVICE_ID    0x0D
#define TMAG3001_REG_MFG_LSB      0x0E  // Manufacturer ID LSB = 0x49
#define TMAG3001_REG_MFG_MSB      0x0F  // Manufacturer ID MSB = 0x54 -> 0x5449
#define TMAG3001_REG_TEMP_MSB     0x10
#define TMAG3001_REG_X_MSB        0x12  // Read 7 bytes: X/Y/Z plus Conv_Status
#define TMAG3001_REG_CONV_STATUS  0x18
#define TMAG3001_REG_STATUS       TMAG3001_REG_CONV_STATUS
#define TMAG3001_REG_DEV_STAT     0x1C

// Device_Config_1: CRC disabled, standard 3-byte I2C read, 1x averaging.
#define TMAG3001_DEV_CFG1_DEFAULT       0x00

// Device_Config_2
#define TMAG3001_DEV_CFG2_STANDBY       0x00
#define TMAG3001_DEV_CFG2_CONTINUOUS    0x02
#define TMAG3001_DEV_CFG2_LOW_NOISE     0x10

// Sensor_Config_1: MAG_CH_EN lives in bits [7:4].
#define TMAG3001_SENS_CFG1_XYZ_EN       0x70
#define TMAG3001_SENS_CFG1_TEMP_EN      0x80
#define TMAG3001_SENS_XYZ_EN            TMAG3001_SENS_CFG1_XYZ_EN
#define TMAG3001_SENS_TEMP_EN           TMAG3001_SENS_CFG1_TEMP_EN

// Sensor_Config_2/3 and INT_Config_1 defaults: raw XYZ, default ranges,
// no angle calculation, no thresholds/wake-on-change, no interrupt output.
#define TMAG3001_SENS_CFG2_RAW_DEFAULT  0x00
#define TMAG3001_SENS_CFG3_DEFAULT      0x00
#define TMAG3001_INT_CFG1_DISABLED      0x00

// Operating modes for DEV_CFG2
#define TMAG3001_MODE_CONT              TMAG3001_DEV_CFG2_CONTINUOUS

// Magnetic range (LSB/mT) - set via SENS_CFG2
#define TMAG3001_RANGE_40MT   885
#define TMAG3001_RANGE_80MT   446
#define TMAG3001_RANGE_120MT  273
#define TMAG3001_RANGE_240MT  137

// Status bits
#define TMAG3001_STATUS_DRDY   0x01  // Conv_Status.Result_Status
#define TMAG3001_STATUS_OVF    0x02  // Magnetic overflow
#define TMAG3001_STATUS_POR    0x10  // Conv_Status.POR

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr7;  // 7-bit I2C address
} tmag3001_t;

typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
    int16_t temp;  // Temperature (LSB, °C = temp/128 + 25 roughly)
    uint8_t status;
} tmag3001_data_t;

// API
HAL_StatusTypeDef TMAG3001_Init(tmag3001_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);
HAL_StatusTypeDef TMAG3001_Probe(I2C_HandleTypeDef *hi2c, uint8_t addr7);
HAL_StatusTypeDef TMAG3001_ReadData(tmag3001_t *dev, tmag3001_data_t *out);
HAL_StatusTypeDef TMAG3001_ReadConfig(tmag3001_t *dev, uint8_t *buf, uint16_t len);
uint8_t TMAG3001_IsDataReady(tmag3001_t *dev);

// Utility: set new I2C address (call before instantiating second device on same bus)
HAL_StatusTypeDef TMAG3001_SetAddress(tmag3001_t *dev, uint8_t new_addr7);

#endif
