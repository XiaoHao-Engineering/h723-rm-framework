/**
 * @file power_bench.h
 * @brief 功率控制实验台 App —— 实验一: 空载扫转速(阶梯保持版)
 *
 * 这个 app 把"实验一"做成一个可以在 MC02 上直接跑的状态机:
 *
 *   WAIT_KEY --按键--> IDLE(2s, 电流0) --自动--> RUN(逐档保持) --> STOP(电流0) --> WAIT_KEY
 *
 * 每一档就是"发一个固定的力矩电流指令, 保持 hold_ms", 期间 50Hz 往串口打一行 CSV:
 *
 *   t_ms, Icmd, rpm, given, temp, vbat_mv, step, pin_mw
 *
 * 为什么是"阶梯"而不是"斜坡":
 *   没有 INA226 的时候, 电流得靠人拿万用表/电源屏去读, 必须让电流"停在某一档"
 *   足够久(默认 8~10s), 人才来得及读数。有传感器时把 hold_ms 调小、步数调多即可。
 *
 * 为什么不用 VOFA+:
 *   数据要和日志共用 USART1, FireWater 混进日志会解析失败;
 *   CSV 行本身可以直接被 tools/power_bench_log.py 存成文件, 再喂给拟合脚本。
 */

#ifndef POWER_BENCH_H
#define POWER_BENCH_H

#include <stdint.h>

/* 最多几档电流(上升 n 档 + 原路返回 n 档 = 2n 个保持段) */
#define PB_STEP_MAX 8

/**
 * @brief 一档: 电流指令 + 保持时间
 *
 * 建议最低档从 500 counts 起: M3508(带减速箱)的静摩擦比较大,
 * 300 counts 以下经常只听到"哒哒"不转。
 */
typedef struct
{
    int32_t i_cmd;   /* 力矩电流指令, counts, 范围 ±16384 */
    uint32_t hold_ms; /* 该档保持时间, 建议 8000~12000 */
} PB_Step_t;

typedef struct
{
    PB_Step_t step[PB_STEP_MAX];
    uint8_t step_cnt;      /* 实际档数 1~PB_STEP_MAX */

    uint8_t down_pass;     /* 1: 上升段跑完再原路返回(查迟滞, 并让数据点翻倍) */
    uint32_t idle_ms;      /* 按键后先空转等这么久, 让轮子彻底停下, 默认 2000 */

    uint16_t log_period_ms; /* CSV 周期, 默认 20ms = 50Hz */
    int32_t rpm_limit;      /* 超速保护, 默认 9000 rpm(转子) */
    int32_t temp_limit;     /* 过温保护, 默认 70 摄氏度 */
} PowerBench_Config_s;

/**
 * @brief 初始化实验台 app(在 RobotInit 中调用)
 *
 * 会注册 1 台 M3508(C620, CAN1, ID=1), 工作在**开环电流模式**:
 * 应用层发多少 counts, 电调就吃多少 counts, 不经过框架的速度/电流 PID。
 * 这样 CSV 里的 Icmd 就是我们自己发的值, 和功率模型里的 I 一一对应。
 */
void PowerBenchInit(void);

/**
 * @brief 实验台周期任务, 在 ROBOT 任务里以 100Hz 调用
 */
void PowerBenchTask(void);

/**
 * @brief 预留: 挂一个"读输入功率"的函数(单位 mW), 没有传感器就不挂
 *
 * 以后加了 INA226 / 功率计, 只要写一个 int32_t xxx(void) 返回 mW,
 * 在这里注册一下, CSV 的 pin_mw 列就有值了, 拟合脚本会自动用它。
 */
void PowerBenchSetPinHook(int32_t (*fn)(void));

#endif /* POWER_BENCH_H */
