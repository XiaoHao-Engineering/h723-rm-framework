/**
 * @file dm_motor.h
 * @brief 达妙(DAMIAO)关节电机模块
 *
 * 协议要点(详见达妙电机手册 DM-J4310/4340P-2EC V1.4):
 *   MIT 模式   : 帧 ID = CAN_ID        , 8 字节 p(16b) v(12b) kp(12b) kd(12b) t(12b)
 *   位置速度   : 帧 ID = 0x100 + CAN_ID, D[0..3] 位置 float(rad)  D[4..7] 限速 float(rad/s)
 *   速度模式   : 帧 ID = 0x200 + CAN_ID, D[0..3] 速度 float(rad/s)
 *   力位混控   : 帧 ID = 0x300 + CAN_ID, D[0..3] 位置 float, D[4..5] 限速 u16 x100, D[6..7] 电流标幺 u16 x10000
 *   使能/失能  : 数据段 = 0xFF...FF + 0xFC / 0xFD, 帧 ID 与"当前模式的控制帧"相同
 *   参数读写   : 帧 ID = 0x7FF 广播, D[0..1]=CANID(小端) D[2]=0x33读 / 0x55写 D[3]=RID
 *   存储参数   : 帧 ID = 0x7FF, D[2]=0xAA D[3]=0x01 (只在失能下生效, 约 30ms)
 *   反馈帧     : 8 字节 id(4b) err(4b) pos(16b) vel(12b) torque(12b) T_MOS(8b) T_Rotor(8b)
 *
 * 重要:
 *   1. 反馈帧里位置/速度/扭矩都是【减速器输出轴】的量, 单位 rad / rad/s / Nm,
 *      不需要再除减速比。4310 输出侧最高 20.9 rad/s(200rpm), 4340P 只有 5.86 rad/s(56rpm)。
 *   2. 位置/速度/扭矩的量程(PMAX/VMAX/TMAX)必须【主机与电机内部一致】,
 *      否则力矩会按比例错(错 4 倍也不会报错)。默认按出厂 ±12.5 / ±30 / ±10 配。
 *   3. 反馈帧 ID 是电机里的 Master ID。本工程的约定是【Master ID = CAN ID】,
 *      初始化时按 motor_id 注册接收, 所以每台电机上电前要在上位机里把 Master ID 改成自己的 ID。
 */

#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include "bsp_can.h"
#include "controller.h"
#include "motor_def.h"
#include "stdint.h"
#include "daemon.h"

#define DM_MOTOR_CNT 12

/* ---------------- 控制模式(写 0x0A 寄存器) ---------------- */
typedef enum
{
    DM_MODE_MIT = 1,       // MIT: p/v/kp/kd/t_ff
    DM_MODE_POS_VEL = 2,   // 位置速度: 位置 + 限速
    DM_MODE_VEL = 3,       // 速度: 只给目标转速
    DM_MODE_POS_FORCE = 4, // 力位混控(PVT): 位置 + 限速 + 电流标幺
} DMMotor_Mode_e;

/* ---------------- 常用寄存器地址(见手册"寄存器列表") ---------------- */
#define DM_RX_ID_AUTO 0xFFu /* can_init_config.rx_id = 它 -> 自动取 motor_id(反馈帧=Master ID) */
#define DM_RID_UV 0x00        // 欠压保护     float
#define DM_RID_KT 0x01        // 扭矩系数     float
#define DM_RID_OT 0x02        // 过温保护     float
#define DM_RID_OC 0x03        // 过流保护(最大相电流标幺, 默认0.8) float
#define DM_RID_ACC 0x04       // 加速度       float
#define DM_RID_DEC 0x05       // 减速度       float(负)
#define DM_RID_MAX_SPD 0x06   // 转子最大速度 float
#define DM_RID_MST_ID 0x07    // 反馈帧 ID    uint32
#define DM_RID_ESC_ID 0x08    // 接收帧 ID    uint32
#define DM_RID_TIMEOUT 0x09   // CAN 超时     uint32(单位 50us)
#define DM_RID_CTRL_MODE 0x0A // 控制模式     uint32 1~4
#define DM_RID_PMAX 0x15      // 位置量程     float
#define DM_RID_VMAX 0x16      // 速度量程     float
#define DM_RID_TMAX 0x17      // 扭矩量程     float
#define DM_RID_OV 0x1D        // 过压保护     float

/* 使能/失能等命令的最后一字节 */
#define DM_CMD_ENABLE 0xFC
#define DM_CMD_DISABLE 0xFD
#define DM_CMD_SAVE_ZERO 0xFE
#define DM_CMD_CLEAR_ERR 0xFB

/* MIT 模式的默认量程(出厂预设), 只是兜底值, 实际请按电机读回值配置 */
#define DM_DEFAULT_P_MIN (-12.5f)
#define DM_DEFAULT_P_MAX (12.5f)
#define DM_DEFAULT_V_MIN (-30.0f)
#define DM_DEFAULT_V_MAX (30.0f)
#define DM_DEFAULT_KP_MIN 0.0f
#define DM_DEFAULT_KP_MAX 500.0f
#define DM_DEFAULT_KD_MIN 0.0f
#define DM_DEFAULT_KD_MAX 5.0f
#define DM_DEFAULT_T_MIN (-10.0f)
#define DM_DEFAULT_T_MAX (10.0f)

/* MIT 模式下的量程设置 */
typedef struct
{
    float p_min, p_max;   // 位置范围(rad)
    float v_min, v_max;   // 速度范围(rad/s)
    float kp_min, kp_max; // 位置增益范围
    float kd_min, kd_max; // 速度增益范围
    float t_min, t_max;   // 力矩范围(Nm)
} DMMotor_Range_s;

/* 达妙电机反馈信息 */
typedef struct
{
    uint8_t state;             // ERR: 0 失能, 1 使能, 3~E 见手册故障码
    float position;            // rad   (输出轴, 单圈, ±PMAX)
    float position_deg;        // 输出轴绝对角度(度), 为了和 DJI 电机的 PID 参数含义保持一致
    float velocity;            // rad/s (输出轴)
    float velocity_deg;        // 度/秒, 为了和 DJI 电机速度环参数含义保持一致
    float torque;              // Nm    (输出轴)
    uint8_t temperature_mos;   // MOS 温度
    uint8_t temperature_rotor; // 线圈温度
} DMMotor_Measure_s;

/**
 * @brief 达妙电机实例
 */
typedef struct
{
    DMMotor_Measure_s measure;              // 电机测量值
    Motor_Control_Setting_s motor_settings; // 电机设置
    Motor_Controller_s motor_controller;    // 电机控制器(仅 MIT 模式使用)

    CANInstance *motor_can_instance; // 电机CAN实例
    DMMotor_Range_s range;           // MIT 量程

    uint16_t motor_id;              // 电机 CAN ID(1~15)
    float kp, kd;
    float mit_torque_ff; /* MIT 位置环模式下的力矩前馈(N·m) */                   // MIT 模式 kp/kd, 都为 0 时是纯力矩模式
    Motor_Type_e motor_type;        // 电机类型
    Motor_Working_Type_e stop_flag; // 启停标志

    uint8_t ctrl_mode;   // 当前控制模式(DMMotor_Mode_e), 决定 DMMotorControl 发哪种控制帧
    float cmd_velocity;  // 速度模式给定 rad/s
    float cmd_position;  // 位置速度模式给定 rad
    float cmd_vel_limit; // 位置速度模式限速 rad/s

    uint8_t mode_readback;      // 从 0x0A 回读到的控制模式(用来确认切模式成功)
    uint32_t param_guard_tick;  // 参数读写后的屏蔽窗口起点(ms), 期间不解析反馈

    DaemonInstance *daemon;
    uint32_t feed_cnt;
    float dt;
} DMMotorInstance;

/**
 * @brief 达妙电机初始化配置
 */
typedef struct
{
    Motor_Controller_Init_s controller_param_init_config;
    Motor_Control_Setting_s controller_setting_init_config;
    DMMotor_Range_s range; // 不设置时使用默认量程(必须在结构体清零后手动赋值)
    uint32_t motor_id;     // 电机 ID, 1~15
    float kp;              // >0 时使用电机内部位置环, =0 时力矩模式
    float kd;
    uint8_t ctrl_mode; // 不填(0)按 MIT 处理, 否则填 DMMotor_Mode_e
    CAN_Init_Config_s can_init_config;
} DMMotor_Init_Config_s;

/**
 * @brief 注册一个达妙电机
 */
DMMotorInstance *DMMotorInit(DMMotor_Init_Config_s *config);

/**
 * @brief 给同一台电机再挂一个接收帧 ID 别名
 *
 * 达妙反馈帧落哪个 ID 取决于电机里的 Master ID(可能 0 / =CAN ID)，而驱动是按
 * 载荷 D[0..1] 认帧的。所以除了注册时的 rx_id，再挂 0 / CAN_ID / 0x100+ID / 0x200+ID
 * 几个别名，任何一帧都能收到(参考例程做法)。
 *
 * @return 1=成功 0=失败(该 rx_id 已被占用等)
 */
uint8_t DMMotorAttachAlias(DMMotorInstance *motor, uint32_t alt_rx_id);

/**
 * @brief 设置电机参考值(闭环目标), 与 DJI 电机一致
 */
void DMMotorSetRef(DMMotorInstance *motor, float ref);

/**
 * @brief 直接以 MIT 模式下发一帧控制(会立即发送)
 */
void DMMotorSetMIT(DMMotorInstance *motor, float position, float velocity,
                   float kp, float kd, float torque_ff);

/**
 * @brief 以"电机内部 MIT 位置环"方式设目标(适合需要刚性+前馈的关节)
 *
 * @param pos_deg   目标角度(度), 内部会换算成 rad 并按 ±PMAX 打包
 * @param kp        位置刚度(0~500, 达妙映射范围)
 * @param kd        阻尼(0~5)
 * @param torque_ff 力矩前馈(N·m), 用来抵重力/摩擦
 */
void DMMotorSetMITCmd(DMMotorInstance *motor, float pos_deg, float kp, float kd, float torque_ff);

/**
 * @brief 设置速度模式的目标转速(rad/s, 输出轴)。
 *        只是记到实例里, 真正的帧由 DMMotorControl() 在 MOTORTASK(1kHz) 里发,
 *        所以每 1ms 都会自动重发, 不需要应用层自己发。
 */
void DMMotorSetVelocity(DMMotorInstance *motor, float rad_s);

/**
 * @brief 设置位置速度模式的目标位置(rad)与限速(rad/s)
 */
void DMMotorSetPosVel(DMMotorInstance *motor, float pos_rad, float vel_limit);

void DMMotorEnable(DMMotorInstance *motor);
void DMMotorDisable(DMMotorInstance *motor);
void DMMotorEnableAt(DMMotorInstance *motor, uint32_t frame_id); // 指定帧ID使能(速度模式下要用 0x200+ID)
void DMMotorSetZero(DMMotorInstance *motor);                     // 把当前位置设为电机零点(会掉电保存)
void DMMotorClearError(DMMotorInstance *motor);                  // 清除电机错误

/**
 * @brief 切换控制模式(写 0x0A)。
 * @note  手册 4.5: 通过 CAN 改模式【不会存 flash】, 掉电回到上次保存的模式,
 *        而且切换瞬间会把位置/速度/力矩指令和 kp/kd 全部清零 -> 切完要重新使能。
 */
void DMMotorSetCtrlMode(DMMotorInstance *motor, uint8_t mode);

/* 参数读写(0x7FF 广播). 写成功后会回一帧同样的数据, 读会回数据 */
void DMMotorWriteParam(DMMotorInstance *motor, uint8_t rid, uint32_t data);
void DMMotorWriteParamFloat(DMMotorInstance *motor, uint8_t rid, float data);
void DMMotorReadParam(DMMotorInstance *motor, uint8_t rid);
void DMMotorSaveParams(DMMotorInstance *motor); // 写 flash, 约 30ms, 只在失能下生效

/**
 * @brief 被 motor_task 调用: 遍历所有达妙电机, 按各自模式计算并发送控制帧
 */
void DMMotorControl(void);

/* 供其他模块读取反馈 */
static inline DMMotor_Measure_s DMMotorGetMeasure(DMMotorInstance *motor)
{
    return motor->measure;
}

/* 是否已使能(ERR==1) */
static inline uint8_t DMMotorIsEnabled(DMMotorInstance *motor)
{
    return (motor->measure.state == 1) ? 1 : 0;
}

/* rad/s <-> rpm (输出轴) */
static inline float DM_RAD2RPM(float rad_s) { return rad_s * 60.0f / 6.2831853f; }
static inline float DM_RPM2RAD(float rpm) { return rpm * 6.2831853f / 60.0f; }

#endif /* DM_MOTOR_H */
