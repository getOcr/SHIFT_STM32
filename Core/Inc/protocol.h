#ifndef SHIFT_PROTOCOL_H
#define SHIFT_PROTOCOL_H

#include <stdint.h>
#include "shift_alert.h"

#define SHIFT_START_BYTE   0xAA
#define SHIFT_END_BYTE     0x55
#define SHIFT_PROTOCOL_VER 0x01

#define SHIFT_FRAME_SIZE   20

typedef struct
{
    uint8_t  start;
    uint8_t  version;

    uint16_t bpm;
    uint16_t spo2;
    int16_t  temperature;

    uint32_t alert_flags;

    uint32_t signal_quality;

    uint16_t sequence;

    uint8_t  crc;
    uint8_t  end;

} SHIFT_Frame;

void SHIFT_BuildFrame(
    SHIFT_Frame *frame,
    float bpm,
    float spo2,
    float temperature_c,
    float signal_quality,
    SHIFT_AlertFlags alerts);

#endif