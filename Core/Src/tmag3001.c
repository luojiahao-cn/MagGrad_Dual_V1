#include "tmag3001.h"
#include <stdio.h>

// TMAG3001 standard register access uses a write of the register pointer
// followed by a repeated START read. The device auto-increments the pointer.
#define TMAG_WRITE_TIMEOUT_MS  50
#define TMAG_READ_TIMEOUT_MS   50

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

static HAL_StatusTypeDef read_mfg_id(tmag3001_t *dev, uint16_t *out_mfg)
{
    uint8_t mfg_data[2];
    HAL_StatusTypeDef s = read_regs(dev, TMAG3001_REG_MFG_LSB, mfg_data, sizeof(mfg_data));
    if (s != HAL_OK) {
        return s;
    }

    *out_mfg = ((uint16_t)mfg_data[1] << 8) | mfg_data[0];
    return HAL_OK;
}

HAL_StatusTypeDef TMAG3001_Probe(I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    tmag3001_t dev = {
        .hi2c = hi2c,
        .addr7 = addr7
    };
    uint16_t mfg_id;

    if (read_mfg_id(&dev, &mfg_id) != HAL_OK) {
        return HAL_ERROR;
    }

    return (mfg_id == 0x5449) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef TMAG3001_Init(tmag3001_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    dev->hi2c = hi2c;
    dev->addr7 = addr7;

    // Presence check
    uint16_t mfg_id;
    int retries = 15;
    HAL_StatusTypeDef last_status = HAL_OK;
    while (retries-- > 0) {
        last_status = read_mfg_id(dev, &mfg_id);
        if (last_status == HAL_OK && mfg_id == 0x5449) {
            printf("TMAG 0x%02X: MFG OK 0x%04X (retries_left=%d)\r\n", addr7, mfg_id, retries);
            break;
        }
        if (retries % 5 == 0) {
            printf("TMAG 0x%02X: MFG retry %d, status=%d, id=0x%04X\r\n",
                   addr7, retries, (int)last_status, mfg_id);
        }
        HAL_Delay(20);
    }
    if (retries < 0) {
        printf("TMAG 0x%02X: MFG ID FAIL after 15 retries (last_status=%d, id=0x%04X)\r\n",
               addr7, (int)last_status, mfg_id);
        return HAL_ERROR;
    }

    // Write SENS_CFG1 (enable X, Y, Z axes)
    {
        HAL_StatusTypeDef s = write_reg(dev, TMAG3001_REG_SENS_CFG1, TMAG3001_SENS_XYZ_EN);
        if (s != HAL_OK) {
            printf("TMAG 0x%02X: write SENS_CFG1 FAIL (status=%d, i2c_err=0x%08lX)\r\n",
                   addr7, (int)s, (unsigned long)HAL_I2C_GetError(dev->hi2c));
            return HAL_ERROR;
        }
        printf("TMAG 0x%02X: SENS_CFG1 OK\r\n", addr7);
    }

    // Write DEV_CFG2 (continuous conversion mode)
    {
        HAL_StatusTypeDef s = write_reg(dev, TMAG3001_REG_DEV_CFG2, TMAG3001_MODE_CONT);
        if (s != HAL_OK) {
            printf("TMAG 0x%02X: write DEV_CFG2 FAIL (status=%d, i2c_err=0x%08lX)\r\n",
                   addr7, (int)s, (unsigned long)HAL_I2C_GetError(dev->hi2c));
            return HAL_ERROR;
        }
        printf("TMAG 0x%02X: DEV_CFG2 OK\r\n", addr7);
    }

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
