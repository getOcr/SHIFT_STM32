#include "shift_alert.h"

/* Persistence counters */
static uint8_t bpm_high_cnt = 0;
static uint8_t bpm_low_cnt = 0;
static uint8_t spo2_low_cnt = 0;
static uint8_t spo2_critical_cnt = 0;
static uint8_t temp_high_cnt = 0;
static uint8_t temp_low_cnt = 0;
static uint8_t signal_low_cnt = 0;

void SHIFT_CheckAlerts(
    MAX30102_Metrics *cardio_metrics,
    float temperature_c,
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
       FALL DETECTION (Reserved)
       =============================== */

    /* 
       Future integration point:

       if(fall_suspected)
           *flags |= ALERT_FALL_SUSPECTED;

       if(fall_confirmed)
           *flags |= ALERT_FALL_CONFIRMED;
    */
}