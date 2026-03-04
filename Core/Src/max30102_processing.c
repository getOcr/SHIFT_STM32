#include "max30102_processing.h"
#include <math.h>

/* Internal circular buffers */
static uint32_t red_buffer[MAX30102_BUFFER_SIZE];
static uint32_t ir_buffer[MAX30102_BUFFER_SIZE];

static float red_ac[MAX30102_BUFFER_SIZE];
static float ir_ac[MAX30102_BUFFER_SIZE];
static float ir_filtered[MAX30102_BUFFER_SIZE];

/* Debug variables for SWV / ITM plotting */
volatile float MAX30102_dbg_red_raw   = 0.0f;
volatile float MAX30102_dbg_ir_raw    = 0.0f;
volatile float MAX30102_dbg_ir_filt   = 0.0f;

/* circular buffer index (next write position) */
static uint16_t sample_index = 0;
/* becomes 1 once we have filled at least one full window */
static uint8_t buffer_full = 0;
/* total number of samples ever stored (used to detect first full window) */
static uint32_t total_samples = 0;
/* number of new samples since last processing call (for 1 Hz processing) */
static uint16_t samples_since_last_process = 0;

/* ==============================
   Store Samples (called in main loop)
   ============================== */
void MAX30102_StoreSample(uint32_t red, uint32_t ir)
{
    red_buffer[sample_index] = red;
    ir_buffer[sample_index]  = ir;

    /* Update debug channels with latest raw samples (100 Hz) */
    MAX30102_dbg_red_raw = (float)red;
    MAX30102_dbg_ir_raw  = (float)ir;

    sample_index++;
    if (sample_index >= MAX30102_BUFFER_SIZE)
    {
        sample_index = 0;
    }

    if (total_samples < 0xFFFFFFFFU)
    {
        total_samples++;
    }
    if (total_samples >= MAX30102_BUFFER_SIZE)
    {
        buffer_full = 1;
    }

    if (samples_since_last_process < 0xFFFFU)
    {
        samples_since_last_process++;
    }
}

/* ==============================
   DC Removal
   ============================== */
static float remove_dc(uint32_t *input, float *output)
{
    float mean = 0.0f;
    uint16_t idx;

    /* First pass: compute mean over chronological window (oldest -> newest) */
    idx = sample_index;
    for (uint16_t n = 0; n < MAX30102_BUFFER_SIZE; n++)
    {
        mean += input[idx];
        idx++;
        if (idx >= MAX30102_BUFFER_SIZE)
            idx = 0;
    }

    mean /= MAX30102_BUFFER_SIZE;

    /* Second pass: create a linear, time-ordered AC sequence in output[0..N-1] */
    idx = sample_index;
    for (uint16_t n = 0; n < MAX30102_BUFFER_SIZE; n++)
    {
        output[n] = (float)input[idx] - mean;
        idx++;
        if (idx >= MAX30102_BUFFER_SIZE)
            idx = 0;
    }

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

        /* Expose last filtered sample for SWV (updated each processing call) */
        if (n == (MAX30102_BUFFER_SIZE - 1))
        {
            MAX30102_dbg_ir_filt = y0;
        }

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

    /* ensure we only process roughly once per second */
    if (samples_since_last_process < (uint16_t)MAX30102_SAMPLE_RATE)
        return 0;
    samples_since_last_process = 0;

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
    total_samples = 0;
    samples_since_last_process = 0;
}