#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/* I2C Address (AD0 = GND), 7-bit addr left-shifted for HAL */
#define MPU6050_I2C_ADDR          (0x68U << 1)
#define MPU6050_ADDR              MPU6050_I2C_ADDR

#define MPU6050_WHO_AM_I_VALUE    0x68U

/* Register addresses */
#define MPU6050_WHO_AM_I          0x75U
#define MPU6050_PWR_MGMT_1        0x6BU
#define MPU6050_SMPRT_DIV         0x19U
#define MPU6050_CONFIG            0x1AU
#define MPU6050_GYRO_CONFIG       0x1BU
#define MPU6050_ACCEL_CONFIG      0x1CU
#define MPU6050_INT_PIN_CFG       0x37U
#define MPU6050_INT_ENABLE        0x38U
#define MPU6050_ACCEL_XOUT_H      0x3BU

/* Data ready INT pin (adjust to your schematic) */
#define MPU6050_INT_PORT          GPIOA
#define MPU6050_INT_PIN           GPIO_PIN_0

/* Sensitivity (default ±2g, ±250dps) */
#define MPU6050_ACCEL_SENS_2G     16384.0f
#define MPU6050_GYRO_SENS_250     131.0f

/* Struct used by MPU6050_formatted.c and main.c */
typedef struct
{
    int16_t acc_x_raw;
    int16_t acc_y_raw;
    int16_t acc_z_raw;
    int16_t temperature_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;

    float acc_x;
    float acc_y;
    float acc_z;
    float temperature;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} Struct_MPU6050;

/* Low-level (optional, used by formatted driver) */
void MPU6050_Writebyte(uint8_t reg_addr, uint8_t val);
void MPU6050_Writebytes(uint8_t reg_addr, uint8_t len, uint8_t *data);
void MPU6050_Readbyte(uint8_t reg_addr, uint8_t *data);
void MPU6050_Readbytes(uint8_t reg_addr, uint8_t len, uint8_t *data);

/* Initialization */
void MPU6050_Initialization(void);

/* Data read / convert */
void MPU6050_Get6AxisRawData(Struct_MPU6050 *mpu6050);
void MPU6050_Get_LSB_Sensitivity(uint8_t FS_SCALE_GYRO, uint8_t FS_SCALE_ACC);
void MPU6050_DataConvert(Struct_MPU6050 *mpu6050);
int  MPU6050_DataReady(void);
void MPU6050_ProcessData(Struct_MPU6050 *mpu6050);

#ifdef __cplusplus
}
#endif

#endif /* INC_MPU6050_H_ */
