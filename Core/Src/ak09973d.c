#include "ak09973d.h"

#define AK09973D_IO_TIMEOUT_MS      100U
#define AK09973D_RESET_DELAY_MS     5U
#define AK09973D_FIRST_SAMPLE_MS    120U
#define AK09973D_DRDY_POLL_MS       2U

static HAL_StatusTypeDef write_reg(ak09973d_t *dev, uint8_t reg, uint8_t val)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->addr7 << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, &val, 1,
                             AK09973D_IO_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_reg(ak09973d_t *dev, uint8_t reg, uint8_t *val)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->addr7 << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, val, 1,
                            AK09973D_IO_TIMEOUT_MS);
}

static HAL_StatusTypeDef read_regs(ak09973d_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Read(dev->hi2c, dev->addr7 << 1, reg,
                            I2C_MEMADD_SIZE_8BIT, buf, len,
                            AK09973D_IO_TIMEOUT_MS);
}

static HAL_StatusTypeDef write_regs(ak09973d_t *dev, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    return HAL_I2C_Mem_Write(dev->hi2c, dev->addr7 << 1, reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len,
                             AK09973D_IO_TIMEOUT_MS);
}

static HAL_StatusTypeDef wait_data_ready(ak09973d_t *dev, uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();

    do {
        uint8_t st;
        if (read_reg(dev, AK09973D_REG_ST, &st) == HAL_OK && (st & AK09973D_ST_DRDY) != 0U) {
            return HAL_OK;
        }
        HAL_Delay(AK09973D_DRDY_POLL_MS);
    } while ((HAL_GetTick() - start) < timeout_ms);

    return HAL_TIMEOUT;
}

HAL_StatusTypeDef AK09973D_Probe(I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t *company, uint8_t *device)
{
    ak09973d_t dev = {
        .hi2c = hi2c,
        .addr7 = addr7
    };
    uint8_t wia[4];

    if (read_regs(&dev, AK09973D_REG_WIA, wia, sizeof(wia)) != HAL_OK) {
        return HAL_ERROR;
    }

    if (company != NULL) {
        *company = wia[0];
    }
    if (device != NULL) {
        *device = wia[1];
    }

    return (wia[0] == AK09973D_WIA_COMPANY && wia[1] == AK09973D_WIA_DEVICE) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef AK09973D_SoftReset(ak09973d_t *dev)
{
    if (write_reg(dev, AK09973D_REG_SRST, 0x01) != HAL_OK) {
        return HAL_ERROR;
    }

    HAL_Delay(AK09973D_RESET_DELAY_MS);
    return HAL_OK;
}

HAL_StatusTypeDef AK09973D_SetMode(ak09973d_t *dev, uint8_t cntl2)
{
    return write_reg(dev, AK09973D_REG_CNTL2, cntl2);
}

HAL_StatusTypeDef AK09973D_ReadConfig(ak09973d_t *dev, ak09973d_config_t *cfg)
{
    uint8_t cntl[2];

    if (cfg == NULL) {
        return HAL_ERROR;
    }

    if (read_regs(dev, AK09973D_REG_CNTL1, cntl, sizeof(cntl)) != HAL_OK) {
        return HAL_ERROR;
    }
    if (read_reg(dev, AK09973D_REG_CNTL2, &cfg->cntl2) != HAL_OK) {
        return HAL_ERROR;
    }

    cfg->cntl1_h = cntl[0];
    cfg->cntl1_l = cntl[1];
    return HAL_OK;
}

HAL_StatusTypeDef AK09973D_Init(ak09973d_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7)
{
    return AK09973D_InitWithConfig(dev, hi2c, addr7, AK09973D_MODE_10HZ);
}

HAL_StatusTypeDef AK09973D_InitWithConfig(ak09973d_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr7, uint8_t cntl2)
{
    dev->hi2c = hi2c;
    dev->addr7 = addr7;

    if (AK09973D_SoftReset(dev) != HAL_OK) {
        return HAL_ERROR;
    }

    if (AK09973D_Probe(hi2c, addr7, NULL, NULL) != HAL_OK) {
        return HAL_ERROR;
    }

    const uint8_t cntl1[2] = {0x00, 0x00};
    if (write_regs(dev, AK09973D_REG_CNTL1, cntl1, sizeof(cntl1)) != HAL_OK) {
        return HAL_ERROR;
    }

    if (AK09973D_SetMode(dev, cntl2) != HAL_OK) {
        return HAL_ERROR;
    }

    return wait_data_ready(dev, AK09973D_FIRST_SAMPLE_MS);
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
