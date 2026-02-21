#include "MPU6050.h"
#include "main.h"
/* extern for I2C handle (defined in main.c) */
extern I2C_HandleTypeDef hi2c1;

/*
 * MPU6050.c
 * ------------------------------------------------------------
 * Hardware bring-up oriented driver (I2C + optional INT pin)
 *
 * Notes:
 * - Uses STM32 HAL I2C mem read/write (hi2c1 from CubeMX).
 * - The public API keeps the same function names/signatures as
 *   your existing file, but the internals are organized to
 *   match the style of max30102.c (small static helpers + clear
 *   init + simple "read one sample" path).
 * ------------------------------------------------------------
 */

Struct_MPU6050 MPU6050;

/* -----------------------------
 *  Local state / config
 * ----------------------------- */
static float LSB_Sensitivity_ACC  = 16384.0f; /* default: +/-2g  */
static float LSB_Sensitivity_GYRO = 131.0f;   /* default: +/-250 dps */

#define MPU6050_I2C_TIMEOUT_MS   (HAL_MAX_DELAY)
#define MPU6050_STARTUP_DELAY_MS (50U)

/* -----------------------------
 *  Low-level I2C helpers
 * ----------------------------- */
static HAL_StatusTypeDef MPU6050_WriteReg(uint8_t reg_addr, uint8_t val)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             MPU6050_ADDR,
                             reg_addr,
                             I2C_MEMADD_SIZE_8BIT,
                             &val,
                             1,
                             MPU6050_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef MPU6050_WriteRegs(uint8_t reg_addr, const uint8_t *data, uint8_t len)
{
    return HAL_I2C_Mem_Write(&hi2c1,
                             MPU6050_ADDR,
                             reg_addr,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)data,
                             len,
                             MPU6050_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef MPU6050_ReadReg(uint8_t reg_addr, uint8_t *val)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            MPU6050_ADDR,
                            reg_addr,
                            I2C_MEMADD_SIZE_8BIT,
                            val,
                            1,
                            MPU6050_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef MPU6050_ReadRegs(uint8_t reg_addr, uint8_t *data, uint8_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            MPU6050_ADDR,
                            reg_addr,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            len,
                            MPU6050_I2C_TIMEOUT_MS);
}

/* -----------------------------
 *  Backwards-compatible wrappers
 *  (kept to match your header)
 * ----------------------------- */
void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val)
{
    (void)MPU6050_WriteReg(reg_addr, val);
}

void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t *data)
{
    (void)MPU6050_WriteRegs(reg_addr, data, len);
}

void MPU6050_Readbyte(uint8_t reg_addr, uint8_t *data)
{
    (void)MPU6050_ReadReg(reg_addr, data);
}

void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t *data)
{
    (void)MPU6050_ReadRegs(reg_addr, data, len);
}

/* -----------------------------
 *  Initialization / bring-up
 * ----------------------------- */
HAL_StatusTypeDef MPU6050_Initialization(void)
{
    HAL_StatusTypeDef status;
    uint8_t who_am_i = 0;

    HAL_Delay(MPU6050_STARTUP_DELAY_MS);

    status = MPU6050_ReadReg(MPU6050_WHO_AM_I, &who_am_i);
    if (status != HAL_OK)
    {
        printf("ERROR: I2C read WHO_AM_I failed (status=%d)\n", (int)status);
        return HAL_ERROR;
    }

    if (who_am_i != 0x68U)
    {
        printf("ERROR: MPU6050 WHO_AM_I = 0x%02X (expected 0x68)\n", who_am_i);
        return HAL_ERROR;
    }
    printf("MPU6050 WHO_AM_I = 0x%02X ... OK\n", who_am_i);

    /* Reset device */
    status = MPU6050_WriteReg(MPU6050_PWR_MGMT_1, (1U << 7));
    if (status != HAL_OK) { printf("ERROR: reset failed\n"); return HAL_ERROR; }
    HAL_Delay(100);

    /* Wake up (clear sleep), select internal clock */
    status = MPU6050_WriteReg(MPU6050_PWR_MGMT_1, 0x00);
    if (status != HAL_OK) { printf("ERROR: wake failed\n"); return HAL_ERROR; }
    HAL_Delay(50);

    /* Sample rate divider:
     * Sample Rate = Gyro Output Rate / (1 + SMPRT_DIV)
     * (Gyro output rate is 8 kHz when DLPF disabled, else 1 kHz)
     */
    status = MPU6050_WriteReg(MPU6050_SMPRT_DIV, 39); /* ~200 Hz when base is 8 kHz */
    if (status != HAL_OK) { printf("ERROR: SMPRT_DIV write failed\n"); return HAL_ERROR; }
    HAL_Delay(10);

    /* DLPF / FSYNC config (0x00 = DLPF disabled / cfg=0) */
    status = MPU6050_WriteReg(MPU6050_CONFIG, 0x00);
    if (status != HAL_OK) { printf("ERROR: CONFIG write failed\n"); return HAL_ERROR; }
    HAL_Delay(10);

    /* Gyro full scale:
     * 0: +/-250 dps, 1: +/-500 dps, 2: +/-1000 dps, 3: +/-2000 dps
     */
    {
        uint8_t FS_SCALE_GYRO = 0x0;
        status = MPU6050_WriteReg(MPU6050_GYRO_CONFIG, (uint8_t)(FS_SCALE_GYRO << 3));
        if (status != HAL_OK) { printf("ERROR: GYRO_CONFIG write failed\n"); return HAL_ERROR; }
        HAL_Delay(10);

        /* Accel full scale:
         * 0: +/-2g, 1: +/-4g, 2: +/-8g, 3: +/-16g
         */
        uint8_t FS_SCALE_ACC = 0x0;
        status = MPU6050_WriteReg(MPU6050_ACCEL_CONFIG, (uint8_t)(FS_SCALE_ACC << 3));
        if (status != HAL_OK) { printf("ERROR: ACCEL_CONFIG write failed\n"); return HAL_ERROR; }
        HAL_Delay(10);

        MPU6050_Get_LSB_Sensitivity(FS_SCALE_GYRO, FS_SCALE_ACC);
        printf("LSB_Sensitivity_GYRO: %f, LSB_Sensitivity_ACC: %f\n", LSB_Sensitivity_GYRO, LSB_Sensitivity_ACC);
    }

    /* Interrupt pin config */
    {
        uint8_t INT_LEVEL    = 0x0; /* 0: active high, 1: active low */
        uint8_t LATCH_INT_EN = 0x0; /* 0: 50us pulse, 1: latch until cleared */
        uint8_t INT_RD_CLEAR = 0x1; /* 1: cleared by any read */

        status = MPU6050_WriteReg(MPU6050_INT_PIN_CFG,
                                  (uint8_t)((INT_LEVEL << 7) |
                                            (LATCH_INT_EN << 5) |
                                            (INT_RD_CLEAR << 4)));
        if (status != HAL_OK) { printf("ERROR: INT_PIN_CFG write failed\n"); return HAL_ERROR; }
        HAL_Delay(10);

        /* Enable Data Ready interrupt */
        status = MPU6050_WriteReg(MPU6050_INT_ENABLE, 0x01);
        if (status != HAL_OK) { printf("ERROR: INT_ENABLE write failed\n"); return HAL_ERROR; }
        HAL_Delay(10);
    }

    printf("MPU6050 initialization finished\n");
    return HAL_OK;
}

/* -----------------------------
 *  Data read + convert
 * ----------------------------- */

/* Read raw accel/gyro/temp registers into the provided struct */
void MPU6050_Get6AxisRawData(Struct_MPU6050 *mpu6050)
{
    uint8_t data[14];

    if (MPU6050_ReadRegs(MPU6050_ACCEL_XOUT_H, data, sizeof(data)) != HAL_OK)
        return;

    mpu6050->acc_x_raw       = (int16_t)((data[0] << 8) | data[1]);
    mpu6050->acc_y_raw       = (int16_t)((data[2] << 8) | data[3]);
    mpu6050->acc_z_raw       = (int16_t)((data[4] << 8) | data[5]);
    mpu6050->temperature_raw = (int16_t)((data[6] << 8) | data[7]);
    mpu6050->gyro_x_raw      = (int16_t)((data[8] << 8) | data[9]);
    mpu6050->gyro_y_raw      = (int16_t)((data[10] << 8) | data[11]);
    mpu6050->gyro_z_raw      = (int16_t)((data[12] << 8) | data[13]);
}

void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC)
{
    switch (FS_SCALE_GYRO)
    {
        case 0: LSB_Sensitivity_GYRO = 131.0f; break;
        case 1: LSB_Sensitivity_GYRO = 65.5f;  break;
        case 2: LSB_Sensitivity_GYRO = 32.8f;  break;
        case 3: LSB_Sensitivity_GYRO = 16.4f;  break;
        default: LSB_Sensitivity_GYRO = 131.0f; break;
    }

    switch (FS_SCALE_ACC)
    {
        case 0: LSB_Sensitivity_ACC = 16384.0f; break;
        case 1: LSB_Sensitivity_ACC = 8192.0f;  break;
        case 2: LSB_Sensitivity_ACC = 4096.0f;  break;
        case 3: LSB_Sensitivity_ACC = 2048.0f;  break;
        default: LSB_Sensitivity_ACC = 16384.0f; break;
    }
}

/* Convert raw -> engineering units: accel in g, gyro in dps, temp in degC */
void MPU6050_DataConvert(Struct_MPU6050 *mpu6050)
{
    mpu6050->acc_x = (float)mpu6050->acc_x_raw / LSB_Sensitivity_ACC;
    mpu6050->acc_y = (float)mpu6050->acc_y_raw / LSB_Sensitivity_ACC;
    mpu6050->acc_z = (float)mpu6050->acc_z_raw / LSB_Sensitivity_ACC;

    mpu6050->temperature = ((float)mpu6050->temperature_raw / 340.0f) + 36.53f;

    mpu6050->gyro_x = (float)mpu6050->gyro_x_raw / LSB_Sensitivity_GYRO;
    mpu6050->gyro_y = (float)mpu6050->gyro_y_raw / LSB_Sensitivity_GYRO;
    mpu6050->gyro_z = (float)mpu6050->gyro_z_raw / LSB_Sensitivity_GYRO;
}

/*
 * Returns 1 when INT pin is high (Data Ready), else 0
 */
/*int MPU6050_DataReady(void)
{
    return (HAL_GPIO_ReadPin(MPU6050_INT_PORT, MPU6050_INT_PIN) == GPIO_PIN_SET) ? 1 : 0;
}
*/

/* One-call helper: read raw + convert */
void MPU6050_ProcessData(Struct_MPU6050 *mpu6050)
{
    MPU6050_Get6AxisRawData(mpu6050);
    MPU6050_DataConvert(mpu6050);
}
