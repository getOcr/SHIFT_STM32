#include "max30102_processing.h"
#include <math.h>

/* Internal circular buffers */
static uint32_t red_buffer[MAX30102_BUFFER_SIZE];
static uint32_t ir_buffer[MAX30102_BUFFER_SIZE];

static float red_ac[MAX30102_BUFFER_SIZE];
static float ir_ac[MAX30102_BUFFER_SIZE];
static float ir_filtered[MAX30102_BUFFER_SIZE];

static uint16_t sample_index = 0;
static uint8_t buffer_full = 0;

/* ==============================
   Store Samples (called in main loop)
   ============================== */
void MAX30102_StoreSample(uint32_t red, uint32_t ir)
{
    red_buffer[sample_index] = red;
    ir_buffer[sample_index]  = ir;

    sample_index++;

    if(sample_index >= MAX30102_BUFFER_SIZE)
    {
        sample_index = 0;
        buffer_full = 1;
    }
}

/* ==============================
   DC Removal
   ============================== */
static float remove_dc(uint32_t *input, float *output)
{
    float mean = 0.0f;

    for(uint16_t i = 0; i < MAX30102_BUFFER_SIZE; i++)
        mean += input[i];

    mean /= MAX30102_BUFFER_SIZE;

    for(uint16_t i = 0; i < MAX30102_BUFFER_SIZE; i++)
        output[i] = (float)input[i] - mean;

    return mean;
}

/* ==============================
   Bandpass Filter (0.5–4 Hz)
   ============================== */
static void bandpass_filter(float *input, float *output)
{
    const float a1 = -1.7786f;
    const float a2 = 0.8008f;
    const float b0 = 0.1006f;
    const float b1 = 0.0f;
    const float b2 = -0.1006f;

    float x1=0,x2=0,y1=0,y2=0;

    for(uint16_t n=0; n<MAX30102_BUFFER_SIZE; n++)
    {
        float x0 = input[n];
        float y0 = b0*x0 + b1*x1 + b2*x2
                   - a1*y1 - a2*y2;

        output[n] = y0;

        x2=x1; x1=x0;
        y2=y1; y1=y0;
    }
}

/* ==============================
   Peak Detection
   ============================== */
static float calculate_bpm(float *signal)
{
    float max_val = 0.0f;
    for(uint16_t i=0;i<MAX30102_BUFFER_SIZE;i++)
        if(signal[i] > max_val)
            max_val = signal[i];

    float threshold = 0.5f * max_val;

    uint16_t peak_count = 0;
    uint16_t last_peak = 0;

    for(uint16_t i=1;i<MAX30102_BUFFER_SIZE-1;i++)
    {
        if(signal[i] > threshold &&
           signal[i] > signal[i-1] &&
           signal[i] > signal[i+1])
        {
            if(i - last_peak > (MAX30102_SAMPLE_RATE * 0.4f))
            {
                peak_count++;
                last_peak = i;
            }
        }
    }

    float duration = MAX30102_BUFFER_SIZE / MAX30102_SAMPLE_RATE;

    return (peak_count / duration) * 60.0f;
}

/* ==============================
   SpO2 Calculation
   ============================== */
static float calculate_spo2(float red_dc, float ir_dc)
{
    float red_rms = 0.0f;
    float ir_rms  = 0.0f;

    for(uint16_t i=0;i<MAX30102_BUFFER_SIZE;i++)
    {
        red_rms += red_ac[i]*red_ac[i];
        ir_rms  += ir_ac[i]*ir_ac[i];
    }

    red_rms = sqrtf(red_rms / MAX30102_BUFFER_SIZE);
    ir_rms  = sqrtf(ir_rms  / MAX30102_BUFFER_SIZE);

    float R = (red_rms / red_dc) / (ir_rms / ir_dc);

    float spo2 = 110.0f - 25.0f * R;

    if(spo2 > 100.0f) spo2 = 100.0f;
    if(spo2 < 70.0f)  spo2 = 70.0f;

    return spo2;
}

/* ==============================
   Signal Quality
   ============================== */
static float compute_signal_quality(void)
{
    float energy = 0.0f;

    for(uint16_t i=0;i<MAX30102_BUFFER_SIZE;i++)
        energy += fabsf(ir_filtered[i]);

    return energy / MAX30102_BUFFER_SIZE;
}

/* ==============================
   Main Processing Function
   ============================== */
uint8_t MAX30102_Process(MAX30102_Metrics *metrics)
{
    if(!buffer_full)
        return 0;

    buffer_full = 0;

    float red_dc = remove_dc(red_buffer, red_ac);
    float ir_dc  = remove_dc(ir_buffer,  ir_ac);

    bandpass_filter(ir_ac, ir_filtered);

    metrics->bpm  = calculate_bpm(ir_filtered);
    metrics->spo2 = calculate_spo2(red_dc, ir_dc);
    metrics->signal_quality = compute_signal_quality();

    return 1;
}

/* ==============================
   Reset
   ============================== */
void MAX30102_ResetBuffer(void)
{
    sample_index = 0;
    buffer_full  = 0;
}