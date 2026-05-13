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
#define AK09973D_MODE_SINGLE   0x01  // Single measurement
#define AK09973D_MODE_10HZ    0x04  // Continuous 10 Hz
#define AK09973D_MODE_100HZ   0x0A  // Continuous 100 Hz

// CNTL2 Bits
#define AK09973D_CNTL2_SMR    (1<<6)  // Sensitivity mode: 0=high, 1=wide range
#define AK09973D_CNTL2_SDR    (1<<5)  // Noise: 0=low noise, 1=low power

// Status bits
#define AK09973D_ST_DRDY      0x01  // Data ready
#define AK09973D_ST_ERR       0x20  // Magnetic overflow

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

// API
HAL_StatusTypeDef AK09973D_Init(ak09973d_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7);
HAL_StatusTypeDef AK09973D_ReadMagData(ak09973d_t *dev, ak09973d_magdata_t *out);
uint8_t AK09973D_IsDataReady(ak09973d_t *dev);

#endif
