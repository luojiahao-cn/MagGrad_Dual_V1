#include "ak09973d.h"

static HAL_StatusTypeDef write_reg(ak09973d_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->addr7 << 1, buf, 2, 100);
}

static HAL_StatusTypeDef read_reg(ak09973d_t *dev, uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Master_Transmit(dev->hi2c, dev->addr7 << 1, &reg, 1, 100) == HAL_OK
           ? HAL_I2C_Master_Receive(dev->hi2c, dev->addr7 << 1, val, 1, 100)
           : HAL_ERROR;
}

// Read multiple bytes starting from reg
static HAL_StatusTypeDef read_regs(ak09973d_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (HAL_I2C_Master_Transmit(dev->hi2c, dev->addr7 << 1, &reg, 1, 100) != HAL_OK)
        return HAL_ERROR;
    return HAL_I2C_Master_Receive(dev->hi2c, dev->addr7 << 1, buf, len, 100);
}

HAL_StatusTypeDef AK09973D_Init(ak09973d_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    dev->hi2c = hi2c;
    dev->addr7 = addr7;

    // Soft reset
    if (write_reg(dev, AK09973D_REG_SRST, 0x01) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(10);

    // Verify WHO_AM_I
    uint8_t wia[4];
    if (read_regs(dev, AK09973D_REG_WIA, wia, 4) != HAL_OK)
        return HAL_ERROR;
    if (wia[0] != AK09973D_WIA_COMPANY || wia[1] != AK09973D_WIA_DEVICE)
        return HAL_ERROR;

    // Set continuous mode 10 Hz
    if (write_reg(dev, AK09973D_REG_CNTL2, AK09973D_MODE_10HZ) != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef AK09973D_ReadMagData(ak09973d_t *dev, ak09973d_magdata_t *out)
{
    uint8_t buf[7];
    if (read_regs(dev, AK09973D_REG_READ_XYZ, buf, 7) != HAL_OK)
        return HAL_ERROR;

    out->status = buf[0];
    out->hz = (int16_t)((buf[1] << 8) | buf[2]);
    out->hy = (int16_t)((buf[3] << 8) | buf[4]);
    out->hx = (int16_t)((buf[5] << 8) | buf[6]);

    return HAL_OK;
}

uint8_t AK09973D_IsDataReady(ak09973d_t *dev)
{
    uint8_t st;
    if (read_reg(dev, AK09973D_REG_ST, &st) == HAL_OK)
        return (st & AK09973D_ST_DRDY) ? 1 : 0;
    return 0;
}
