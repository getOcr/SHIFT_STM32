#include "AHT21.h"
#include <string.h>

uint8_t  AHT_21_ADDR      = (0x38 << 1);
uint32_t i2c_RETRY_TIME   = 100;

static I2C_HandleTypeDef *aht21_hi2c = NULL;

static HAL_StatusTypeDef AHT21_Write(uint8_t *data, uint16_t len)
{
    if (aht21_hi2c == NULL) return HAL_ERROR;
    return HAL_I2C_Master_Transmit(aht21_hi2c, AHT_21_ADDR, data, len, i2c_RETRY_TIME);
}

static HAL_StatusTypeDef AHT21_Read(uint8_t *data, uint16_t len)
{
    if (aht21_hi2c == NULL) return HAL_ERROR;
    return HAL_I2C_Master_Receive(aht21_hi2c, AHT_21_ADDR, data, len, i2c_RETRY_TIME);
}

HAL_StatusTypeDef AHT21_Init(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef ret;
    uint8_t buff[1];

    aht21_hi2c = hi2c;

    // Check device presence
    ret = HAL_I2C_IsDeviceReady(aht21_hi2c, AHT_21_ADDR, 2, i2c_RETRY_TIME);
    if (ret != HAL_OK) return HAL_ERROR;

    // Read status byte (0x71)
    buff[0] = 0x71;
    ret = AHT21_Write(buff, 1);
    if (ret != HAL_OK) return HAL_ERROR;

    ret = AHT21_Read(buff, 1);
    if (ret != HAL_OK) return HAL_ERROR;

    // Check calibration bits (typical check used in many AHT2x drivers)
    if ( (buff[0] & 0x18) != 0x18 )
        return HAL_ERROR;

    return HAL_OK;
}

HAL_StatusTypeDef AHT21_init(void)
{
#ifdef AHT21_I2C_PORT
    return AHT21_Init(&AHT21_I2C_PORT);
#else
    // If AHT21_I2C_PORT isn't defined, you must call AHT21_Init(&hi2cX)
    return HAL_ERROR;
#endif
}

// -----------------------------------------------------------------------------
// AHT21 measurement (humidity + temperature share same command)
// -----------------------------------------------------------------------------
static HAL_StatusTypeDef AHT21_TriggerMeasurement(void)
{
    uint8_t cmd[3] = { 0xAC, 0x33, 0x00 };
    return AHT21_Write(cmd, 3);
}

static HAL_StatusTypeDef AHT21_Read6(uint8_t buff[6])
{
    return AHT21_Read(buff, 6);
}


uint32_t AHT21_Read_Humidity(void)
{
    HAL_StatusTypeDef ret;
    uint8_t buff[6];
    uint32_t humidity_raw;
    uint32_t humidity_pct;

    ret = AHT21_TriggerMeasurement();
    if (ret != HAL_OK) return 0xFFFFFFFF;

    HAL_Delay(100);

    ret = AHT21_Read6(buff);
    if (ret != HAL_OK) return 0xFFFFFFFF;

    // Humidity is 20-bit: buff[1..3]
    humidity_raw = ((uint32_t)buff[1] << 12) | ((uint32_t)buff[2] << 4) | ((uint32_t)buff[3] >> 4);

    // %RH = raw * 100 / 2^20
    humidity_pct = (humidity_raw * 100U) / 0x100000U;

    return humidity_pct;
}

int32_t AHT21_Read_Temperature(void)
{
    HAL_StatusTypeDef ret;
    uint8_t buff[6];
    uint32_t temp_raw;
    int32_t temp_c;

    ret = AHT21_TriggerMeasurement();
    if (ret != HAL_OK) return (int32_t)0x80000000; // INT32_MIN-ish sentinel

    HAL_Delay(100);

    ret = AHT21_Read6(buff);
    if (ret != HAL_OK) return (int32_t)0x80000000;

    // Temperature is 20-bit: buff[3..5] (lower nibble of buff[3])
    temp_raw = (((uint32_t)(buff[3] & 0x0F)) << 16) | ((uint32_t)buff[4] << 8) | (uint32_t)buff[5];

    // T(C) = raw * 200 / 2^20 - 50
    temp_c = (int32_t)((temp_raw * 200U) / 0x100000U) - 50;

    return temp_c;
}
