#ifndef POWER_TEST_H
#define POWER_TEST_H

#include <stdint.h>

/* 日志格式：
 *   0 = CSV（带表头，给 Python 拟合脚本用）
 *   1 = FireWater（纯文本 "a,b,c,d\n"，VOFA+ 最省事）
 *   2 = JustFloat（4 个 float + 0x00 0x00 0x80 0x7F，高速稳定）
 */
#ifndef PT_LOG_MODE
#define PT_LOG_MODE 1
#endif

/* 实验参数 */
typedef struct {
    uint8_t  motor_id;       /* C620 的 ID，1~4 */
    int32_t  i_target;       /* 目标电流指令（counts），实验一建议 1500~2500 */
    uint32_t idle_ms;        /* 上电后等待时间，默认 2000 */
    uint32_t ramp_ms;        /* 电流爬升时间，默认 60000 */
    uint32_t hold_ms;        /* 到顶后保持时间，默认 3000 */
    uint16_t log_period_ms;  /* 日志周期，默认 5（200 Hz） */
    int16_t  rpm_limit;      /* 超速保护，默认 9000 */
    int8_t   temp_limit;     /* 过温保护，默认 80 */
} pt_config_t;

/* 电机实时状态 */
typedef struct {
    int16_t  speed_rpm;      /* 转子转速 rpm */
    int16_t  given_current;  /* C620 回传的力矩电流 */
    uint8_t  temperature;    /* 电机温度 °C */
    uint16_t bus_mv;         /* 母线电压 mV */
    int32_t  bus_ma;         /* 母线电流 mA（INA226） */
    uint8_t  online;         /* 收到过反馈 */
} pt_motor_t;

void pt_init(const pt_config_t *cfg);
void pt_start(void);
void pt_stop(void);
void pt_tick_1ms(void);                                  /* 1 kHz 调用 */
void pt_on_can_rx(uint32_t id, const uint8_t data[8], uint8_t len);
const pt_motor_t *pt_motor(void);
const pt_config_t *pt_config(void);

/* ===== 移植层必须实现 ===== */
void     pt_port_can_send(uint32_t id, const uint8_t data[8]);
uint32_t pt_port_now_ms(void);
void     pt_port_log(const char *s);                     /* 文本输出 */
void     pt_port_write(const uint8_t *buf, uint16_t len);/* 二进制输出（JustFloat） */
void     pt_port_read_bus(uint16_t *mv, int32_t *ma);    /* 没有 INA226 就返回 0 */

#endif
