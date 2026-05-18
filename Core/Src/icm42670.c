#include "icm42670.h"
#include "csv_writer.h"
#include <string.h>
#include <stdio.h>

static inline void CS_L(icm42670_t *c) { HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_RESET); }
static inline void CS_H(icm42670_t *c) { HAL_GPIO_WritePin(c->cs_port, c->cs_pin, GPIO_PIN_SET); }

/* 单寄存器/突发读写：在同一次 CS 低电平的事务内完成（符合集成电路时序） */
static HAL_StatusTypeDef spi_rw(icm42670_t *c, uint8_t reg, uint8_t *buf, uint16_t n, uint8_t rd)
{
    uint8_t hdr = rd ? (uint8_t)(0x80 | (reg & 0x7F)) : (uint8_t)(reg & 0x7F);
    HAL_StatusTypeDef st;
    CS_L(c);
    if (HAL_SPI_Transmit(c->hspi, &hdr, 1, 100) != HAL_OK)
    {
        CS_H(c);
        return HAL_ERROR;
    }
    st = rd ? HAL_SPI_Receive(c->hspi, buf, n, 100) : HAL_SPI_Transmit(c->hspi, buf, n, 100);
    CS_H(c);
    return st;
}
static HAL_StatusTypeDef w8(icm42670_t *c, uint8_t reg, uint8_t v) { return spi_rw(c, reg, &v, 1, 0); }
static HAL_StatusTypeDef r1(icm42670_t *c, uint8_t reg, uint8_t *v) { return spi_rw(c, reg, v, 1, 1); }
static HAL_StatusTypeDef rN(icm42670_t *c, uint8_t reg, uint8_t *buf, uint16_t n) { return spi_rw(c, reg, buf, n, 1); }

HAL_StatusTypeDef ICM42670_Init(icm42670_t *icm)
{
    /* 1) 软件复位 */
    if (w8(icm, 0x02, 0x10) != HAL_OK)
        return HAL_ERROR;
    HAL_Delay(30);

    /* 2) 接口配置：4-wire SPI + Mode0/3 */
    if (w8(icm, 0x01, 0x04) != HAL_OK)
        return HAL_ERROR;

    /* 3) 核对芯片 ID */
    uint8_t who = 0;
    if (r1(icm, 0x75, &who) != HAL_OK || who != 0x67)
        return HAL_ERROR;

    /* 4) 量程/ODR：GYRO=±1000 dps, ODR=200 Hz；ACC=±4 g, ODR=200 Hz */
    if (w8(icm, 0x20, (1u << 5) | 0x08) != HAL_OK)
        return HAL_ERROR; // GYRO_CONFIG0
    if (w8(icm, 0x21, (2u << 5) | 0x08) != HAL_OK)
        return HAL_ERROR; // ACCEL_CONFIG0

    /* 5) 打开六轴低噪声模式 */
    if (w8(icm, 0x1F, 0x0F) != HAL_OK)
        return HAL_ERROR; // PWR_MGMT0
    HAL_Delay(45);

    return HAL_OK;
}

HAL_StatusTypeDef ICM42670_ReadRaw(icm42670_t *icm, icm42670_raw_t *out)
{
    /* 连续读 TEMP(0x09..0x0A)+ACC(0x0B..0x10)+GYR(0x11..0x16) 共14字节 */
    uint8_t b[14];
    if (rN(icm, 0x09, b, sizeof b) != HAL_OK)
        return HAL_ERROR;

    out->temp = (int16_t)((b[0] << 8) | b[1]);
    out->ax = (int16_t)((b[2] << 8) | b[3]);
    out->ay = (int16_t)((b[4] << 8) | b[5]);
    out->az = (int16_t)((b[6] << 8) | b[7]);
    out->gx = (int16_t)((b[8] << 8) | b[9]);
    out->gy = (int16_t)((b[10] << 8) | b[11]);
    out->gz = (int16_t)((b[12] << 8) | b[13]);
    return HAL_OK;
}

int ICM42670_ReadToCSV(icm42670_t *icm, char *out, size_t out_size)
{
    icm42670_raw_t d;
    size_t off = 0;

    if (ICM42670_ReadRaw(icm, &d) != HAL_OK) {
        if (!CSV_AppendString(out, out_size, &off, "ICMERR,READ\r\n")) {
            return 0;
        }
        return (int)off;
    }

    if (!CSV_AppendString(out, out_size, &off, "ICM,") ||
        !CSV_AppendI32(out, out_size, &off, d.ax) ||
        !CSV_AppendChar(out, out_size, &off, ',') ||
        !CSV_AppendI32(out, out_size, &off, d.ay) ||
        !CSV_AppendChar(out, out_size, &off, ',') ||
        !CSV_AppendI32(out, out_size, &off, d.az) ||
        !CSV_AppendChar(out, out_size, &off, ',') ||
        !CSV_AppendI32(out, out_size, &off, d.gx) ||
        !CSV_AppendChar(out, out_size, &off, ',') ||
        !CSV_AppendI32(out, out_size, &off, d.gy) ||
        !CSV_AppendChar(out, out_size, &off, ',') ||
        !CSV_AppendI32(out, out_size, &off, d.gz) ||
        !CSV_AppendChar(out, out_size, &off, ',') ||
        !CSV_AppendI32(out, out_size, &off, d.temp) ||
        !CSV_AppendCRLF(out, out_size, &off)) {
        return 0;
    }

    return (int)off;
}

void ICM42670_ReadAndPrint(icm42670_t *icm)
{
    icm42670_raw_t d;
    if (ICM42670_ReadRaw(icm, &d) == HAL_OK)
    {
        printf("ICM,%d,%d,%d,%d,%d,%d,%d\r\n",
               d.ax, d.ay, d.az, d.gx, d.gy, d.gz, d.temp);
    }
    else
    {
        printf("ICMERR,READ\r\n");
    }
}
