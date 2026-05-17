#ifndef __AK09973D_H
#define __AK09973D_H

#include "stm32h7xx_hal.h"
#include <stdint.h>

// I2C Addresses
#define AK09973D_ADDR_1        0x10  // IF1=OD-INT, IF2=SDA
#define AK09973D_ADDR_2        0x11  // IF1=SDA, IF2=OD-INT

// Registers
#define AK09973D_REG_WIA       0x00  // Company/Device ID
#define AK09973D_REG_ST        0x10  // Status
#define AK09973D_REG_READ_XYZ  0x17  //连续读7字节: ST,HZ,HY,HX
#define AK09973D_REG_CNTL1     0x20
#define AK09973D_REG_CNTL2     0x21
#define AK09973D_REG_SRST      0x30  // Soft reset

// ID Values
#define AK09973D_WIA_COMPANY   0x48
#define AK09973D_WIA_DEVICE    0xC1

// CNTL2 Modes
#define AK09973D_MODE_POWERDOWN 0x00
#define AK09973D_MODE_SINGLE   0x01  // Single measurement
#define AK09973D_MODE_5HZ      0x02  // Continuous 5 Hz
#define AK09973D_MODE_10HZ    0x04  // Continuous 10 Hz
#define AK09973D_MODE_20HZ     0x06  // Continuous 20 Hz
#define AK09973D_MODE_50HZ     0x08  // Continuous 50 Hz
#define AK09973D_MODE_100HZ   0x0A  // Continuous 100 Hz
#define AK09973D_MODE_500HZ    0x0C  // Continuous 500 Hz
#define AK09973D_MODE_1000HZ   0x0E  // Continuous 1000 Hz
#define AK09973D_MODE_2000HZ   0x10  // Continuous 2000 Hz

// CNTL2 Bits
#define AK09973D_CNTL2_SELF   (1U<<7) // Self-test enable
#define AK09973D_CNTL2_SMR    (1U<<6) // 0=high sensitivity, 1=wide range
#define AK09973D_CNTL2_SDR    (1U<<5) // 0=low noise, 1=low power
#define AK09973D_CNTL2_MODE_MASK 0x1F

// Status bits
#define AK09973D_ST_DRDY      0x01  // Data ready
#define AK09973D_ST_DOR       0x40  // Data overrun
#define AK09973D_ST_ERR       0x20  // Magnetic overflow
#define AK09973D_ST_FIXED     0x80  // Always reads 1

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t addr7;  // 7-bit I2C address
} ak09973d_t;

typedef struct {
    int16_t hx;
    int16_t hy;
    int16_t hz;
    uint8_t status;
} ak09973d_magdata_t;

typedef struct {
    uint8_t cntl1_l;
    uint8_t cntl1_h;
    uint8_t cntl2;
} ak09973d_config_t;

// API
HAL_StatusTypeDef AK09973D_Init(ak09973d_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);
HAL_StatusTypeDef AK09973D_InitWithConfig(ak09973d_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t cntl2);
HAL_StatusTypeDef AK09973D_Probe(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t *company, uint8_t *device);
HAL_StatusTypeDef AK09973D_SoftReset(ak09973d_t *dev);
HAL_StatusTypeDef AK09973D_SetMode(ak09973d_t *dev, uint8_t cntl2);
HAL_StatusTypeDef AK09973D_ReadConfig(ak09973d_t *dev, ak09973d_config_t *cfg);
HAL_StatusTypeDef AK09973D_ReadMagData(ak09973d_t *dev, ak09973d_magdata_t *out);
uint8_t AK09973D_IsDataReady(ak09973d_t *dev);

#endif
