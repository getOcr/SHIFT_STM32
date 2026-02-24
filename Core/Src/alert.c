#include "alert.h"
#include <math.h>
/* Persistence counters */
static uint8_t bpm_high_cnt = 0;
static uint8_t bpm_low_cnt = 0;
static uint8_t spo2_low_cnt = 0;
static uint8_t spo2_critical_cnt = 0;
static uint8_t temp_high_cnt = 0;
static uint8_t temp_low_cnt = 0;
static uint8_t signal_low_cnt = 0;
static float pre_ax = 0.0f, pre_ay = 0.0f, pre_az = 0.0f;
static float pre_mag = 1.0f;
static uint32_t fall_timestamp = 0;
static uint8_t fall_state = 0;

void SHIFT_CheckAlerts(
    MAX30102_Metrics *cardio_metrics,
    float temperature_c,
    Struct_MPU6050 *mpu6050,
    SHIFT_AlertFlags *flags)
{
    *flags = 0;

    /* ===============================
       HEART RATE
       =============================== */

    if(cardio_metrics->bpm > BPM_HIGH_THRESHOLD)
    {
        if(++bpm_high_cnt >= TRIP_COUNT_REQUIRED)
            *flags |= ALERT_BPM_HIGH;
    }
    else bpm_high_cnt = 0;

    if(cardio_metrics->bpm < BPM_LOW_THRESHOLD)
    {
        if(++bpm_low_cnt >= TRIP_COUNT_REQUIRED)
            *flags |= ALERT_BPM_LOW;
    }
    else bpm_low_cnt = 0;

    /* ===============================
       SPO2
       =============================== */

    if(cardio_metrics->spo2 < SPO2_LOW_THRESHOLD)
    {
        if(++spo2_low_cnt >= TRIP_COUNT_REQUIRED)
            *flags |= ALERT_SPO2_LOW;
    }
    else spo2_low_cnt = 0;

    if(cardio_metrics->spo2 < SPO2_CRITICAL_THRESHOLD)
    {
        if(++spo2_critical_cnt >= TRIP_COUNT_REQUIRED)
            *flags |= ALERT_SPO2_CRITICAL;
    }
    else spo2_critical_cnt = 0;

    /* ===============================
       TEMPERATURE
       =============================== */

    if(temperature_c > TEMP_HIGH_THRESHOLD)
    {
        if(++temp_high_cnt >= TRIP_COUNT_REQUIRED)
            *flags |= ALERT_TEMP_HIGH;
    }
    else temp_high_cnt = 0;

    if(temperature_c < TEMP_LOW_THRESHOLD)
    {
        if(++temp_low_cnt >= TRIP_COUNT_REQUIRED)
            *flags |= ALERT_TEMP_LOW;
    }
    else temp_low_cnt = 0;

    /* ===============================
       SIGNAL QUALITY
       =============================== */

    if(cardio_metrics->signal_quality < SIGNAL_QUALITY_THRESHOLD)
    {
        if(++signal_low_cnt >= TRIP_COUNT_REQUIRED)
            *flags |= ALERT_SIGNAL_LOW;
    }
    else signal_low_cnt = 0;

    /* ===============================
       SENSOR FAULT CHECK
       =============================== */

    if(cardio_metrics->bpm <= 0.0f ||
       cardio_metrics->spo2 <= 0.0f)
    {
        *flags |= ALERT_SENSOR_FAULT;
    }

    /* ===============================
       FALL DETECTION
       =============================== */


    //   Future integration point:
    float ax = mpu6050->acc_x;
    float ay = mpu6050->acc_y;
    float az = mpu6050->acc_z;
    float current_mag = sqrtf(ax*ax + ay*ay + az*az);

    if(fall_state == 0)
    {
        if (current_mag > 0.8f && current_mag < 1.2f)
        {
            pre_ax = ax;
            pre_ay = ay;
            pre_az = az;
            pre_mag = current_mag;
        }

        if (current_mag > 3.0f)
        {
            fall_state = 1;
            fall_timestamp = HAL_GetTick();
            *flags |= ALERT_FALL_SUSPECTED;
        }
    }

    else if (fall_state == 1)
    {
        if (HAL_GetTick() - fall_timestamp >= 2000)
        {
            float dot_product = (pre_ax * ax + pre_ay * ay + pre_az * az);
            float cos_theta = dot_product / (pre_mag * current_mag);

            if (cos_theta > 1.0f) cos_theta = 1.0f;
            if (cos_theta < -1.0f) cos_theta = -1.0f;

            //float angle_change = acosf(cos_theta) * 57.29578f;

            if (cos_theta < 0.707f)
            {
                *flags |= ALERT_FALL_CONFIRMED;
            }

            fall_state = 0;
        }
    }

}