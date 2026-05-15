#include "tmag3001.h"
#include <stdio.h>

// TMAG3001 I2C Protocol:
// Note: TMAG3001 requires I2C at 100kHz (not 400kHz)

#define TMAG_WRITE_TIMEOUT_MS  1000
#define TMAG_READ_TIMEOUT_MS   1000

static HAL_StatusTypeDef write_reg(tmag3001_t *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->addr7 << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, &val, 1, TMAG_WRITE_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_reg(tmag3001_t *dev, uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->addr7 << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1, TMAG_READ_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_regs(tmag3001_t *dev, uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->addr7 << 1, start_reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len, TMAG_READ_TIMEOUT_MS);
}

// Read MFG ID using separate tx/rx (required for TMAG3001)
static HAL_StatusTypeDef read_mfg_id(tmag3001_t *dev, uint16_t *out_mfg)
{
    uint8_t reg_addr = TMAG3001_REG_MFG_LSB;
    uint8_t mfg_data[2];
    HAL_StatusTypeDef status;

    status = HAL_I2C_Master_Transmit(dev->hi2c, dev->addr7 << 1, &reg_addr, 1, TMAG_WRITE_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf("TMAG: I2C TX failed (addr=0x%02X, err=%d)\r\n", dev->addr7, status);
        return HAL_ERROR;
    }

    status = HAL_I2C_Master_Receive(dev->hi2c, (dev->addr7 << 1) | 1, mfg_data, 2, TMAG_READ_TIMEOUT_MS);
    if (status != HAL_OK) {
        printf("TMAG: I2C RX failed (addr=0x%02X, err=%d)\r\n", dev->addr7, status);
        return HAL_ERROR;
    }

    *out_mfg = ((uint16_t)mfg_data[1] << 8) | mfg_data[0];
    return HAL_OK;
}

HAL_StatusTypeDef TMAG3001_Init(tmag3001_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    dev->hi2c = hi2c;
    dev->addr7 = addr7;

    // Retry MFG ID read several times
    uint16_t mfg_id;
    int retries = 10;
    while (retries-- > 0) {
        HAL_StatusTypeDef status = read_mfg_id(dev, &mfg_id);
        if (status == HAL_OK && mfg_id == 0x5449) {
            break;
        }
        HAL_Delay(20);
    }
    if (retries < 0) {
        printf("TMAG Init: MFG ID failed (addr=0x%02X)\r\n", addr7);
        return HAL_ERROR;
    }

    // Write SENS_CFG1 (enable X, Y, Z axes)
    if (write_reg(dev, TMAG3001_REG_SENS_CFG1, TMAG3001_SENS_XYZ_EN) != HAL_OK)
        return HAL_ERROR;

    // Write DEV_CFG2 (continuous conversion mode)
    if (write_reg(dev, TMAG3001_REG_DEV_CFG2, TMAG3001_MODE_CONT) != HAL_OK)
        return HAL_ERROR;

    // Wait for first conversion to complete
    HAL_Delay(50);

    return HAL_OK;
}

HAL_StatusTypeDef TMAG3001_ReadData(tmag3001_t *dev, tmag3001_data_t *out)
{
    // Read 7 bytes: X_MSB, X_LSB, Y_MSB, Y_LSB, Z_MSB, Z_LSB, Conv_Status
    uint8_t buf[7];
    if (read_regs(dev, TMAG3001_REG_X_MSB, buf, sizeof(buf)) != HAL_OK)
        return HAL_ERROR;

    out->x = (int16_t)((buf[0] << 8) | buf[1]);
    out->y = (int16_t)((buf[2] << 8) | buf[3]);
    out->z = (int16_t)((buf[4] << 8) | buf[5]);
    out->status = buf[6];

    return HAL_OK;
}

uint8_t TMAG3001_IsDataReady(tmag3001_t *dev)
{
    uint8_t status;
    if (read_reg(dev, TMAG3001_REG_STATUS, &status) == HAL_OK)
        return (status & TMAG3001_STATUS_DRDY) ? 1 : 0;
    return 0;
}

HAL_StatusTypeDef TMAG3001_SetAddress(tmag3001_t *dev, uint8_t new_addr7)
{
    uint8_t val = (new_addr7 << 1) | 0x01;
    return write_reg(dev, TMAG3001_REG_I2C_ADDR, val);
}
