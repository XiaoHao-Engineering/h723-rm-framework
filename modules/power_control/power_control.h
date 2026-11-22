#ifndef POWER_CONTROL_H
#define POWER_CONTROL_H

#include "dji_motor.h"

/**
 * 底盘功率控制（修正版）
 *
 * 与开源工程的差异：
 *   1. k1 乘 ω²、k2 乘 I² —— 原工程把这两个系数挂反了
 *   2. 判别式 < 0 时把力矩压到 0，而不是放行原指令
 *   3. 不直接改写 PID 的 out，而是改写 current_cmd（dji_motor 在发 CAN 前留的钩子）
 *   4. 功率再分配、负功率跳过、限幅、按方向选根，逻辑与原文一致
 */

/* 单台电机的功率模型参数 */
typedef struct
{
    float k1; // W/rpm²，乘 ω²
    float k2; // W/count²，乘 Icmd²
    float a;  // W，静态损耗
    float ct; // W/(rpm·count)，机械功率系数
} PowerModel_s;

typedef struct
{
    DJIMotorInstance **motors; // 参与功率限制的电机
    uint8_t motor_cnt;         // 数量（底盘是 4）
    PowerModel_s model;        // 四轮同型号可共用一组参数

    float default_limit_w; // 裁判没给数据时的默认功率上限
    float buffer_target_j; // 缓冲能量目标，默认 30 J
    float cap_boost_w;     // 电容允许额外出的功率（没电容填 0）

    uint8_t use_referee; // 1：用裁判数据；0：台架模拟（自己积分缓冲能量）
    float buffer_kp;
    float buffer_ki;
} PowerControl_Config_s;

void PowerControlInit(PowerControl_Config_s *cfg);

/* 100 Hz 调用：更新预算
 * use_referee=1 时传入裁判的底盘功率与缓冲能量
 * use_referee=0 时参数被忽略，内部用实测总功率积分出缓冲能量      */
void PowerControlUpdate(float chassis_power, float buffer, float limit_w);

/* 1 kHz 调用：由 dji_motor 的输出钩子自动触发，不需要手动调 */
void PowerControlApply(void);

float PowerControlGetBudget(void);     // 电机侧总预算 W
float PowerControlGetTotalPower(void); // 上一次算出的总功率 W
float PowerControlGetScale(void);      // 上一次的缩放系数 k（1 表示没限）
void PowerControlSetCapBoost(float w); // 动态调整电容额外功率

#endif // POWER_CONTROL_H
