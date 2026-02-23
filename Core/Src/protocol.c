#include "shift_protocol.h"

/* Simple XOR CRC */
static uint8_t SHIFT_ComputeCRC(uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    for(uint16_t i = 0; i < len; i++)
        crc ^= data[i];
    return crc;
}

void SHIFT_BuildFrame(
    SHIFT_Frame *frame,
    float bpm,
    float spo2,
    float temperature_c,
    float signal_quality,
    SHIFT_AlertFlags alerts)
{
    static uint16_t sequence_counter = 0;

    frame->start = SHIFT_START_BYTE;
    frame->version = SHIFT_PROTOCOL_VER;

    // scaled integer approach (reduce parsing complexity)
    frame->bpm = (uint16_t)(bpm * 10.0f);
    frame->spo2 = (uint16_t)(spo2 * 10.0f);
    frame->temperature = (int16_t)(temperature_c * 100.0f);

    frame->alert_flags = alerts;

    frame->signal_quality = (uint32_t)(signal_quality * 10.0f);

    frame->sequence = sequence_counter++;

    frame->crc = SHIFT_ComputeCRC(
        (uint8_t*)frame,
        SHIFT_FRAME_SIZE - 2);

    frame->end = SHIFT_END_BYTE;
}