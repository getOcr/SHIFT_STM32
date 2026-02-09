#ifndef MAX30102_H
#define MAX30102_H

#include "main.h"

#define MAX30102_I2C_ADDR        (0x57U << 1) // 7-bit address left-shifted by 1 for I2C

/* Register Map */
#define REG_INTR_STATUS_1        0x00U   
#define REG_INTR_STATUS_2        0x01U
#define REG_FIFO_WR_PTR          0x04U
#define REG_FIFO_RD_PTR          0x06U
#define REG_FIFO_DATA            0x07U  // 
#define REG_FIFO_CONFIG          0x08U
#define REG_MODE_CONFIG          0x09U
#define REG_SPO2_CONFIG          0x0AU
#define REG_LED1_PA              0x0CU
#define REG_LED2_PA              0x0DU

/* Mode Configuration (0x09): SHDN(7) | RESET(6) | reserved(5~3) | MODE(2~0) */
#define MODE_SHDN_BIT            7U
#define MODE_RESET_BIT           6U
#define MODE_HR_ONLY             0x02U   /* Heart rate only */
#define MODE_SPO2                0x03U   /* Heart rate and SpO2 (what we want)*/

/*
 * SpO2 Configuration Register (REG ADDR 0x0A, POR 0x00, R/W)
 * Bit layout: reserved(7) | SPO2_ADC_RGE[1:0](6:5) | SPO2_SR[2:0](4:2) | LED_PW[1:0](1:0)
 *
 * Table 5 - SpO2 ADC Range Control (18-Bit Resolution), bits 6:5:
 *   SPO2_ADC_RGE[1:0] = 00 -> LSB 7.81 pA,  FULL SCALE 2048 nA
 *   SPO2_ADC_RGE[1:0] = 01 -> LSB 15.63 pA, FULL SCALE 4096 nA
 *   SPO2_ADC_RGE[1:0] = 10 -> LSB 31.25 pA, FULL SCALE 8192 nA
 *   SPO2_ADC_RGE[1:0] = 11 -> LSB 62.5 pA,  FULL SCALE 16384 nA
 *
 * Table 6 - SpO2 Sample Rate Control (bits 4:2), one sample = one IR + one Red conversion:
 *   SPO2_SR[2:0] = 000 ->  50 sps
 *   SPO2_SR[2:0] = 001 -> 100 sps
 *   SPO2_SR[2:0] = 010 -> 200 sps
 *   SPO2_SR[2:0] = 011 -> 400 sps
 *   SPO2_SR[2:0] = 100 -> 800 sps
 *   SPO2_SR[2:0] = 101 -> 1000 sps
 *   SPO2_SR[2:0] = 110 -> 1600 sps
 *   SPO2_SR[2:0] = 111 -> 3200 sps
 *   Note: Sample rate upper bound depends on LED_PW; see Table 11/12.
 *
 * Table 7 - LED Pulse Width Control (bits 1:0):
 *   LED_PW[1:0] = 00 ->  69 µs (68.95 µs),  ADC 15 bits
 *   LED_PW[1:0] = 01 -> 118 µs (117.78 µs), ADC 16 bits
 *   LED_PW[1:0] = 10 -> 215 µs (215.44 µs), ADC 17 bits
 *   LED_PW[1:0] = 11 -> 411 µs (410.75 µs), ADC 18 bits
 */
#define SPO2_ADC_RGE_MASK        0x60U   /* bits 6:5 */
#define SPO2_SR_MASK              0x1CU   /* bits 4:2 */
#define LED_PW_MASK               0x03U   /* bits 1:0 */
#define SPO2_ADC_RGE_VAL          0x00U   /* 00 -> 2048 nA full scale (Table 5) */
#define SPO2_SR_100HZ             0x01U   /* 001 -> 100 sps (Table 6) */
#define LED_PW_411US              0x03U   /* 11 -> 411 µs, 18-bit ADC (Table 7) */

/*  6 = Red×3 + IR×3） */
#define FIFO_SAMPLE_BYTES        6U

extern I2C_HandleTypeDef *max30102_hi2c;

HAL_StatusTypeDef MAX30102_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MAX30102_ReadFIFO_OneSample(uint32_t *red, uint32_t *ir);
HAL_StatusTypeDef MAX30102_GetFIFOLevel(uint8_t *wr_ptr, uint8_t *rd_ptr);
#endif
