#ifndef SHIFT_ALERT_H
#define SHIFT_ALERT_H

#include <stdint.h>
#include "max30102_processing.h"

/* 32-bit system alert register  (may want to decrease to 16) */
typedef uint32_t SHIFT_AlertFlags;

/* Bit Definitions */
#define ALERT_BPM_HIGH           (1UL << 0)
#define ALERT_BPM_LOW            (1UL << 1)
#define ALERT_SPO2_LOW           (1UL << 2)
#define ALERT_SPO2_CRITICAL      (1UL << 3)

#define ALERT_TEMP_HIGH          (1UL << 4)
#define ALERT_TEMP_LOW           (1UL << 5)

#define ALERT_SIGNAL_LOW         (1UL << 6)
#define ALERT_SENSOR_FAULT       (1UL << 7)

/* Reserved for MPU6050 fall detection 
#define ALERT_FALL_SUSPECTED     (1UL << 8)
#define ALERT_FALL_CONFIRMED     (1UL << 9)
*/
/* -------- Thresholds -------- */

/* Cardiovascular */
#define BPM_HIGH_THRESHOLD         140.0f
#define BPM_LOW_THRESHOLD           45.0f

/* Oxygen */
#define SPO2_LOW_THRESHOLD          92.0f
#define SPO2_CRITICAL_THRESHOLD     88.0f

/* Thermal */
#define TEMP_HIGH_THRESHOLD         40.0f   // Celsius
#define TEMP_LOW_THRESHOLD          0.0f

/* Signal Quality */
#define SIGNAL_QUALITY_THRESHOLD    50.0f

/* Persistence */ 
#define TRIP_COUNT_REQUIRED         3


void SHIFT_CheckAlerts(
    MAX30102_Metrics *cardio_metrics,
    float temperature_c,
    SHIFT_AlertFlags *flags);

#endif