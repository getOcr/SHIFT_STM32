#include "max30102.h"

I2C_HandleTypeDef *max30102_hi2c = NULL;

static HAL_StatusTypeDef MAX30102_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return HAL_I2C_Master_Transmit(max30102_hi2c, MAX30102_I2C_ADDR, buf, 2, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef MAX30102_ReadReg(uint8_t reg, uint8_t *value) // read one byte
{
    if (HAL_I2C_Master_Transmit(max30102_hi2c, MAX30102_I2C_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;
    return HAL_I2C_Master_Receive(max30102_hi2c, MAX30102_I2C_ADDR, value, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef MAX30102_ReadRegs(uint8_t reg, uint8_t *data, uint16_t len) // read multiple bytes
{
    if (HAL_I2C_Master_Transmit(max30102_hi2c, MAX30102_I2C_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return HAL_ERROR;
    return HAL_I2C_Master_Receive(max30102_hi2c, MAX30102_I2C_ADDR, data, len, HAL_MAX_DELAY);
}

HAL_StatusTypeDef MAX30102_Init(I2C_HandleTypeDef *hi2c)
{
    max30102_hi2c = hi2c;
    HAL_StatusTypeDef status;

    status = MAX30102_WriteReg(REG_MODE_CONFIG, (1U << MODE_RESET_BIT));
    if (status != HAL_OK) return status;

    HAL_Delay(10);

    status = MAX30102_WriteReg(REG_MODE_CONFIG, MODE_SPO2);
    if (status != HAL_OK) return status;

    status = MAX30102_WriteReg(REG_SPO2_CONFIG,
        (SPO2_ADC_RGE_VAL << 5) | (SPO2_SR_100HZ << 2) | (LED_PW_411US << 0));
    if (status != HAL_OK) return status;

    status = MAX30102_WriteReg(REG_FIFO_CONFIG, 0x4FU);
    if (status != HAL_OK) return status;

    status = MAX30102_WriteReg(REG_LED1_PA, 0x24U);
    if (status != HAL_OK) return status;

    status = MAX30102_WriteReg(REG_LED2_PA, 0x24U);
    if (status != HAL_OK) return status;

    return HAL_OK;
}

HAL_StatusTypeDef MAX30102_GetFIFOLevel(uint8_t *wr_ptr, uint8_t *rd_ptr)
{
    uint8_t w, r;
    if (MAX30102_ReadReg(REG_FIFO_WR_PTR, &w) != HAL_OK) return HAL_ERROR;
    if (MAX30102_ReadReg(REG_FIFO_RD_PTR, &r) != HAL_OK) return HAL_ERROR;
    *wr_ptr = w;
    *rd_ptr = r;
    return HAL_OK;
}

HAL_StatusTypeDef MAX30102_ReadFIFO_OneSample(uint32_t *red, uint32_t *ir)
{
    uint8_t buf[FIFO_SAMPLE_BYTES];
    if (MAX30102_ReadRegs(REG_FIFO_DATA, buf, FIFO_SAMPLE_BYTES) != HAL_OK)
        return HAL_ERROR;

    *red = (uint32_t)buf[0] << 16 | (uint32_t)buf[1] << 8 | buf[2];
    *ir  = (uint32_t)buf[3] << 16 | (uint32_t)buf[4] << 8 | buf[5];
    return HAL_OK;
}
