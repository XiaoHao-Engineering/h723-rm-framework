#include "c620.h"

static void put_be16(uint8_t *p, int16_t v)
{
    p[0] = (uint8_t)((uint16_t)v >> 8);
    p[1] = (uint8_t)((uint16_t)v & 0xFF);
}

static int16_t get_be16(const uint8_t *p)
{
    return (int16_t)(((uint16_t)p[0] << 8) | p[1]);
}

void c620_encode(const int32_t i[4], uint8_t out[8])
{
    for (int k = 0; k < 4; k++) {
        int32_t v = i[k];
        if (v > 16384)  v = 16384;
        if (v < -16384) v = -16384;
        put_be16(&out[k * 2], (int16_t)v);
    }
}

int16_t c620_decode_angle(const uint8_t d[8])   { return get_be16(&d[0]); }
int16_t c620_decode_speed(const uint8_t d[8])   { return get_be16(&d[2]); }
int16_t c620_decode_current(const uint8_t d[8]) { return get_be16(&d[4]); }
uint8_t c620_decode_temp(const uint8_t d[8])    { return d[6]; }
