#include "power_test.h"
#include "c620.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef enum {
    ST_IDLE = 0,
    ST_RAMP,
    ST_HOLD,
    ST_DOWN,
    ST_DONE
} pt_state_t;

static pt_config_t s_cfg;
static pt_motor_t  s_motor;
static pt_state_t  s_state;
static uint32_t    s_t0;
static uint32_t    s_last_log;
static int32_t     s_cmd;
static uint8_t     s_running;
static uint8_t     s_header_done;

static int32_t clamp_i(int32_t v, int32_t lim)
{
    if (v > lim) return lim;
    if (v < -lim) return -lim;
    return v;
}

void pt_init(const pt_config_t *cfg)
{
    s_cfg = *cfg;
    if (s_cfg.motor_id < 1 || s_cfg.motor_id > 4) s_cfg.motor_id = 1;
    if (s_cfg.idle_ms == 0) s_cfg.idle_ms = 2000;
    if (s_cfg.ramp_ms == 0) s_cfg.ramp_ms = 60000;
    if (s_cfg.hold_ms == 0) s_cfg.hold_ms = 3000;
    if (s_cfg.log_period_ms == 0) s_cfg.log_period_ms = 5;
    if (s_cfg.rpm_limit == 0) s_cfg.rpm_limit = 9000;
    if (s_cfg.temp_limit == 0) s_cfg.temp_limit = 80;

    memset(&s_motor, 0, sizeof(s_motor));
    s_state = ST_IDLE;
    s_running = 0;
    s_cmd = 0;
    s_header_done = 0;
}

void pt_start(void)
{
    s_t0 = pt_port_now_ms();
    s_last_log = s_t0;
    s_state = ST_IDLE;
    s_running = 1;
    s_header_done = 0;
}

void pt_stop(void)
{
    s_running = 0;
    s_state = ST_DONE;
    s_cmd = 0;
}

const pt_motor_t *pt_motor(void)   { return &s_motor; }
const pt_config_t *pt_config(void) { return &s_cfg; }

/* ---------------------------------------------------------------- 反馈解析 */
void pt_on_can_rx(uint32_t id, const uint8_t data[8], uint8_t len)
{
    if (len < 7) return;
    if (id != (uint32_t)(0x201 + s_cfg.motor_id - 1)) return;

    s_motor.speed_rpm     = c620_decode_speed(data);
    s_motor.given_current = c620_decode_current(data);
    s_motor.temperature   = data[6];
    s_motor.online        = 1;
}

/* ---------------------------------------------------------------- 1kHz 主循环 */
void pt_tick_1ms(void)
{
    uint32_t now, el, t_ramp_end, t_hold_end, t_down_end;
    int32_t target;

    if (!s_running) return;
    now = pt_port_now_ms();
    el  = now - s_t0;

    target = s_cfg.i_target;
    if (target > 16000)  target = 16000;
    if (target < -16000) target = -16000;

    t_ramp_end = s_cfg.idle_ms + s_cfg.ramp_ms;
    t_hold_end = t_ramp_end + s_cfg.hold_ms;
    t_down_end = t_hold_end + s_cfg.ramp_ms;

    if (el < s_cfg.idle_ms) {
        s_state = ST_IDLE;
        s_cmd = 0;
    } else if (el < t_ramp_end) {
        s_state = ST_RAMP;
        s_cmd = (int32_t)((int64_t)target * (int32_t)(el - s_cfg.idle_ms) / (int32_t)s_cfg.ramp_ms);
    } else if (el < t_hold_end) {
        s_state = ST_HOLD;
        s_cmd = target;
    } else if (el < t_down_end) {
        s_state = ST_DOWN;
        s_cmd = (int32_t)((int64_t)target *
                          (int32_t)(t_down_end - el) / (int32_t)s_cfg.ramp_ms);
    } else {
        s_state = ST_DONE;
        s_cmd = 0;
    }

    /* 保护：超速 / 过温 / 掉线后仍给大力矩 */
    if (s_motor.speed_rpm > s_cfg.rpm_limit ||
        s_motor.temperature > (uint8_t)s_cfg.temp_limit) {
        s_cmd = 0;
        s_state = ST_DONE;
    }

    s_cmd = clamp_i(s_cmd, 16000);

    /* 1 kHz 发一帧：C620 只有收到帧才会更新输出 */
    {
        uint8_t frame[8];
        int32_t out[4] = {0, 0, 0, 0};
        out[s_cfg.motor_id - 1] = s_cmd;
        c620_encode(out, frame);
        pt_port_can_send(0x200, frame);
    }

    /* 日志 */
    if ((uint32_t)(now - s_last_log) >= s_cfg.log_period_ms) {
        uint16_t mv = 0;
        int32_t  ma = 0;
        s_last_log = now;
        pt_port_read_bus(&mv, &ma);
        s_motor.bus_mv = mv;
        s_motor.bus_ma = ma;

#if PT_LOG_MODE == 0
        {
            char line[128];
            if (!s_header_done) {
                pt_port_log("t_ms,Icmd,rpm,given,temp,bus_mv,bus_ma\r\n");
                s_header_done = 1;
            }
            snprintf(line, sizeof(line), "%lu,%ld,%d,%d,%u,%u,%ld\r\n",
                     (unsigned long)el, (long)s_cmd,
                     s_motor.speed_rpm, s_motor.given_current,
                     (unsigned)s_motor.temperature,
                     (unsigned)s_motor.bus_mv, (long)s_motor.bus_ma);
            pt_port_log(line);
        }
#elif PT_LOG_MODE == 1
        {
            /* VOFA+ FireWater：4 通道 Icmd, rpm, 功率mW, 母线mV */
            char line[96];
            int32_t p_mw = (int32_t)((int64_t)mv * ma / 1000);
            snprintf(line, sizeof(line), "%ld,%d,%ld,%u\r\n",
                     (long)s_cmd, s_motor.speed_rpm,
                     (long)p_mw, (unsigned)mv);
            pt_port_log(line);
        }
#else
        {
            /* VOFA+ JustFloat：4 个 float + 帧尾 00 00 80 7F */
            float f[4];
            uint8_t b[20];
            f[0] = (float)s_cmd;
            f[1] = (float)s_motor.speed_rpm;
            f[2] = (float)mv * (float)ma / 1.0e6f;
            f[3] = (float)mv;
            memcpy(b, f, 16);
            b[16] = 0x00; b[17] = 0x00; b[18] = 0x80; b[19] = 0x7F;
            pt_port_write(b, 20);
        }
#endif
    }

    if (s_state == ST_DONE && s_cmd == 0 && (el > t_down_end + 1000)) {
        if (s_running) {
#if PT_LOG_MODE == 0
            pt_port_log("# DONE\r\n");
#endif
            s_running = 0;
        }
    }
}
