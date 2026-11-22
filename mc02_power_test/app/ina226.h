#ifndef INA226_H
#define INA226_H

#include <stdint.h>

/* I2C 读写函数由移植层提供，返回 0 表示成功 */
typedef int (*ina_write_fn)(uint8_t addr, uint8_t reg,
                            const uint8_t *buf, uint16_t len);
typedef int (*ina_read_fn)(uint8_t addr, uint8_t reg,
                           uint8_t *buf, uint16_t len);

/* 默认地址 0x40（A0/A1 都接地） */
int ina226_init(ina_write_fn wr, ina_read_fn rd, uint8_t addr, float r_mohm);

/* 读母线电压 mV 和电流 mA（电流带符号，充电为负） */
int ina226_read(uint16_t *mv, int32_t *ma);

#endif
