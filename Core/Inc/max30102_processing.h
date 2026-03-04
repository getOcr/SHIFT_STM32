#ifndef MAX30102_PROCESSING_H
#define MAX30102_PROCESSING_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#define MAX30102_SAMPLE_RATE     100.0f
/* Sliding window: 8 seconds @ sample rate, processed once per second */
#define MAX30102_BUFFER_SECONDS  8U
#define MAX30102_BUFFER_SIZE     ((uint16_t)(MAX30102_SAMPLE_RATE * (float)MAX30102_BUFFER_SECONDS))

typedef struct
{
    float bpm;
    float spo2;
    float signal_quality;
} MAX30102_Metrics;

/* Call this every time a new FIFO sample is read */
void MAX30102_StoreSample(uint32_t red, uint32_t ir);

/* Returns 1 when buffer full and processed */
uint8_t MAX30102_Process(MAX30102_Metrics *metrics);

/* Reset internal buffer */
void MAX30102_ResetBuffer(void);

#endif