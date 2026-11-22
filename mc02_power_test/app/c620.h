#ifndef C620_H
#define C620_H

#include <stdint.h>

/* 把 4 路力矩电流（-16384~16384）编码成 0x200 的 8 字节，大端 */
void   c620_encode(const int32_t i[4], uint8_t out[8]);

/* 0x201~0x204 反馈解析 */
int16_t  c620_decode_angle(const uint8_t d[8]);    /* 0~8191 */
int16_t  c620_decode_speed(const uint8_t d[8]);    /* rpm */
int16_t  c620_decode_current(const uint8_t d[8]);  /* counts */
uint8_t  c620_decode_temp(const uint8_t d[8]);     /* °C */

#endif
