#ifndef LOADER_HEAT_H
#define LOADER_HEAT_H

#include <stdint.h>

/**
 * @file loader_heat.h
 * @brief 发射热量管理：底层状态机估计 + 上层前馈规划 + 硬限幅
 *
 * 分层（与说明文档一致）：
 *   底层「数」：确认真的打出一发 -> Qest += d；每周期 Qest -= a*dt
 *   上层「算」：按规则不等式反解允许射频（稳态优先时 ≈ a/d）
 *   兜底「限」：Qest 快满时先禁连发、再禁单发
 *
 * 用法（shoot.c 内，ROBOT 任务 100Hz）：
 *   LoaderHeatFeedFriction(aps_l, aps_r, current_a);
 *   LoaderHeatFeedCommand(rps);        // 连发指令射频(发/s)，没有就传 0
 *   LoaderHeatUpdate(now_ms);
 *   rps = LoaderHeatGetAllowedRps();
 */

/* 状态机对外状态（用于日志/上报） */
typedef enum
{
    LOADER_HEAT_IDLE = 0,   /* 摩擦轮没起转 */
    LOADER_HEAT_READY,      /* 已起转，等电流脉冲 */
    LOADER_HEAT_SUSPECT,    /* 电流超阈值，等持续时间 */
    LOADER_HEAT_CONFIRMED,  /* 刚确认一发（瞬态） */
} loader_heat_state_e;

void LoaderHeatInit(void);

/* ---- 输入：每个控制周期喂一次 ---- */
void LoaderHeatFeedFriction(float aps_l, float aps_r, float current_a);
void LoaderHeatFeedCommand(float cmd_rps); /* 连发指令射频(发/s) */
void LoaderHeatOnSingleShot(void);         /* 单发下令时调一次 */

/* ---- 周期更新（100Hz） ---- */
void LoaderHeatUpdate(float now_ms);

/* ---- 输出 ---- */
float LoaderHeatGetQ(void);            /* 当前热量估计 */
float LoaderHeatGetLimit(void);        /* 热量上限 Q0 */
float LoaderHeatGetCoolRps(void);      /* 冷却速率 a */
float LoaderHeatGetPerShot(void);      /* 每发消耗 d */
float LoaderHeatGetSteadyRps(void);    /* a/d 稳态射频 */
float LoaderHeatGetAllowedRps(void);   /* 规划后的允许射频(发/s) */
float LoaderHeatGetCmdRps(void);       /* 当前连发指令射频(发/s) */
uint8_t LoaderHeatIsSingleOnly(void);  /* 1: 只允许单发 */
uint8_t LoaderHeatIsBlocked(void);     /* 1: 禁止发射 */
uint8_t LoaderHeatGetState(void);      /* loader_heat_state_e */
uint32_t LoaderHeatGetConfirmed(void); /* 已确认发数 */
uint32_t LoaderHeatGetMissed(void);    /* 漏检兜底次数 */

#endif /* LOADER_HEAT_H */
