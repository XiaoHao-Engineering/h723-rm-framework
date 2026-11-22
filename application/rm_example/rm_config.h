/**
 * @file rm_config.h
 * @brief 这一版车(RM 折叠步兵)的【全部可调参数】都放在这里
 *
 * 电机与关节:
 *   单 Yaw   : DM-J4310-2EC      MIT 力矩模式 + 框架三环 + IMU 反馈
 *   大 Pitch : DM-J4310P-2EC     MIT 力矩模式 + 框架三环 + 编码器反馈 + 重力前馈
 *   (交叉滚子轴承, 在下, 抬整个云台)
 *   小 Pitch : DM-J4310-2EC      MIT 力矩模式 + 框架三环 + IMU 反馈 + 重力前馈
 *   (在上, 装在大 P 输出上, 调发射仰角)
 *   拨弹盘   : DM-J4310-2EC      速度模式(0x200+ID) + 电流限幅 + 卡弹自恢复
 *   摩擦轮   : 2 x M3508        速度环
 *   底盘     : 4 x M3508 全向轮   (见 chassis_omni/)
 *
 * CAN 分配:
 *   CAN1  底盘 4 x M3508   ID 1/2/3/4
 *   CAN2  云台 3 x DM      ID 1(Yaw) / 2(大P) / 3(小P)
 *   CAN3  拨弹 DM ID 4 + 摩擦轮 2 x M3508 ID 1/2
 *
 * ★ 上机前必做:
 *   1. 用达妙调试助手把每台电机的 Master ID 改成与 CAN ID 相同(本工程按 CAN ID 收反馈);
 *   2. 读 0x17(TMAX) 确认与下面的 DM_TMAX 一致, 不一致就改这里(力矩会成比例错);
 *   3. 拨弹盘要让电机每次上电都切速度模式 -> 代码里已做(0x0A=3), 但需确认串口打印的模式。
 */

#ifndef RM_CONFIG_H
#define RM_CONFIG_H

#include "bsp_board.h"

/* 这一版台架调试没有裁判系统, 关掉 UI 任务(否则 UITask 会访问未初始化的裁判系统指针) */
#define RM_USE_REFEREE (0)
#define RM_USE_OMNI_CHASSIS (0)  /* 1: 用四轮全向轮底盘(chassis_omni) 代替 RM 麦轮底盘(robot.c 里二选一) */
#define RM_LOADER_ANGLE_ENABLE (1)
#define RM_LOADER_POS_MODE (1) /* 1: 单发用位置模式(目标=当前角+30°, 松手保持), 0: 用原来的速度环角度法 */
#define LOADER_POS_KP (3.0f)     /* 软件位置环 P: rpm / 度 */
#define LOADER_POS_MIN_RPM (12.0f) /* 最小爬行速度, 阻力大时也能顶到位 */
#define LOADER_POS_VEL_LIMIT (4.0f) /* 位置模式限速 rad/s(输出轴), 太大会撞, 太小会慢 */
#define LOADER_POS_DONE_DEG (1.5f)  /* 位置误差小于多少度算到位 */
/* 说明: 位置模式保持时, 力矩被电机 OC(过流百分比, 现设 0.14) 限住,
 *       按 TMAX=10Nm 估算上限约 1.4Nm, 低于你例程的 1.6Nm 上限, 所以可以保持。 */
 /* 1: 右上拨杆在上时拨弹盘单发(见 shoot.md) */

/* 视觉: 台架没有视觉, 关掉(不初始化串口, 也不发数据) */
#define RM_USE_VISION (0)

/* ==========================================================================
 * 一、CAN 总线与电机 ID
 * ========================================================================== */
#define RM_CAN_CHASSIS BOARD_CAN1
#define RM_CAN_GIMBAL BOARD_CAN2
#define RM_CAN_SHOOT BOARD_CAN3

#define YAW_CAN_ID (4) /* 挪到 4: CAN2 的 ID1 让给小 Pitch(用户要求) */
#define BIGP_CAN_ID (2)
#define SMALLP_CAN_ID (1) /* 小 Pitch 电机 ID=1 */
/* 小 Pitch 用"电机内部 MIT 位置环"(kp/kd) + 重力前馈:
 * 响应快、刚性好; kp 0~500, kd 0~5 是达妙内部映射范围 */
#define MINI_PITCH_MIT_ENABLE (1)
#define MINI_PITCH_KP (60.0f) /* 位置刚度: 越大越硬, 太大易自激 */
#define MINI_PITCH_KD (2.5f)  /* 阻尼: 抑制过冲和手推摆动 */
#define LOADER_CAN_ID (1)            /* 达妙出厂默认 ID=1(未改过) */
#define LOADER_RX_ID  (0)            /* 达妙反馈帧 ID = 上位机里的 Master ID, 出厂默认 0
                                      * 若收不到反馈: 把上位机 Master ID 改成 1, 或这里改 1 */

/* 摩擦轮是否已接线: 1=已接(会按目标初速转), 0=没接(不发使能) */
#define RM_FRICTION_WHEEL_ENABLE (1)

#define FRICTION_L_ID (2) /* 按实测: 左摩擦轮 C620 ID=2 */
#define FRICTION_R_ID (3) /* 按实测: 右摩擦轮 C620 ID=3 */

/* ==========================================================================
 * 二、DM 电机 MIT 量程(必须与电机内部 PMAX/VMAX/TMAX 一致!)
 *
 * 出厂预设是 ±12.5 rad / ±30 rad/s / ±10 Nm。
 * 4310 峰值 12.5Nm, 但出厂 TMAX 默认 10 -> 只用到 80%。
 * 若你确认电机里 0x17 是 12.5, 把 DM_TMAX 改成 12.5f 可以多出 25% 力矩。
 * ★ 这里改错不会报错, 只会让力矩按比例缩放(错 1.25 倍 / 4 倍), 务必先读回确认。
 * ========================================================================== */
#define DM_PMAX (12.5f)
#define DM_VMAX (30.0f)
#define DM_TMAX (10.0f)

/* ==========================================================================
 * 三、云台几何与限位(度)
 * ========================================================================== */

/* 大 P 编码器读数为 0 时, 炮管的绝对仰角(装配后标定一次)。
 * 大 P 的目标 = 期望绝对仰角 - 这个值。 */
#define BIGP_ALIGN_ABS_DEG (0.0f)

/* Yaw 电机编码器读数为多少度时, 云台与底盘朝向一致(装配后标定一次)。
 * 底盘跟随用的 offset 就是 (当前 yaw 编码器角 - 这个值)。 */
#define YAW_MOTOR_ALIGN_DEG (0.0f)

/* 大 P 机械行程(编码器域) */
#define BIGP_MIN_DEG (-25.0f)
#define BIGP_MAX_DEG (70.0f)

/* 小 P 机械行程(相对大 P) */
#define SMALLP_MIN_DEG (-15.0f)
#define SMALLP_MAX_DEG (15.0f)

/* 炮管绝对仰角指令范围(IMU 域), 遥控器/上层给的角度会被夹在这里 */
#define PITCH_CMD_MIN_DEG (-20.0f)
#define PITCH_CMD_MAX_DEG (70.0f)

/* 云台跟随底盘的死区(度): 偏差小于它时不转, 防止原地抖 */
#define CHASSIS_FOLLOW_DEADBAND_DEG (3.0f)
#define CHASSIS_FOLLOW_GAIN (3.0f)   /* wz = -gain * err */
#define CHASSIS_FOLLOW_MAX_W (6.0f)  /* rad/s */

/* ==========================================================================
 * 四、重力前馈
 *
 * 静态重力矩 tau = m * g * L * cos(theta)
 *   小 P: 只承担炮管+末端机构
 *   大 P: 承担(小 P 那部分 + 大 P 自己的臂)
 * theta 直接用 IMU 的绝对 pitch(小 P 行程只有 ±15°, 对 cos 影响 < 3%, 可以忽略)
 *
 * ★ 符号无法凭空确定: 上机后手动把炮管抬到水平, 看电流是"往上顶"还是"往下压",
 *   不对就把对应的 SIGN 改成 -1。
 * ★ 质量与力臂先按下面的估, 标定方法: 断开 kp 只留前馈, 看炮管是否停在各个角度不动。
 * ========================================================================== */
#define GRAV_FF_ENABLE (1)
#define G_ACC (9.81f)

#define SMALLP_LOAD_MASS (0.45f) /* kg, 小P 之后的所有东西 */
#define SMALLP_LOAD_ARM (0.055f) /* m, 小P 轴到质心 */
#define BIGP_EXTRA_MASS (0.35f)  /* kg, 大P 自己的臂+支架 */
#define BIGP_LOAD_ARM (0.090f)   /* m, 大P 轴到质心 */

#define GRAV_FF_SIGN_SMALL (1.0f)
#define GRAV_FF_SIGN_BIG (1.0f)

/* 前馈上限(Nm), 防止 cos 算错时把电机顶死 */
#define GRAV_FF_MAX_SMALL (2.0f)
#define GRAV_FF_MAX_BIG (5.0f)

/* ==========================================================================
 * 五、云台 PID
 *
 * ★★ 单位提醒(和 DJI 电机最大的区别):
 *     MIT 力矩模式下, 速度环的输出直接就是 t_ff(N·m), 不是电流 counts!
 *     所以速度环 Kp 只有 0.0x 这个量级, 绝对不能照抄 DJI 的 50/200。
 *     角度环输出是"度/秒", 这个和 DJI 一致, 可以沿用。
 * ========================================================================== */

/* Yaw: 角度环(IMU 总角度) -> 速度环(陀螺 z) -> 力矩 */
#define YAW_ANGLE_KP (14.0f)
#define YAW_ANGLE_KI (0.0f)
#define YAW_ANGLE_MAXOUT (400.0f) /* 度/秒 */
#define YAW_ANGLE_DEADBAND (0.3f)
#define YAW_SPEED_KP (0.035f)
#define YAW_SPEED_KI (0.080f)
#define YAW_SPEED_MAXOUT (8.0f) /* N·m */
#define YAW_SPEED_I_LIMIT (4.0f)

/* 大 P: 角度环(编码器) -> 速度环(编码器) -> 力矩 + 重力前馈 */
#define BIGP_ANGLE_KP (10.0f)
#define BIGP_ANGLE_KI (0.0f)
#define BIGP_ANGLE_MAXOUT (200.0f)
#define BIGP_ANGLE_DEADBAND (0.2f)
#define BIGP_SPEED_KP (0.035f)
#define BIGP_SPEED_KI (0.080f)
#define BIGP_SPEED_MAXOUT (6.0f)
#define BIGP_SPEED_I_LIMIT (3.0f)

/* 小 P: 角度环(IMU pitch) -> 速度环(陀螺 x) -> 力矩 + 重力前馈 */
#define SMALLP_ANGLE_KP (12.0f)
#define SMALLP_ANGLE_KI (0.0f)
#define SMALLP_ANGLE_MAXOUT (250.0f)
#define SMALLP_ANGLE_DEADBAND (0.2f)
#define SMALLP_SPEED_KP (0.030f)
#define SMALLP_SPEED_KI (0.060f)
#define SMALLP_SPEED_MAXOUT (4.0f)
#define SMALLP_SPEED_I_LIMIT (2.0f)

/* 云台摇杆速度: 摇杆满杆时每秒转多少度 */
#define YAW_STICK_RATE_DEG_S (180.0f)
#define PITCH_STICK_RATE_DEG_S (90.0f)
#define RC_DEADBAND (20)

/* ==========================================================================
 * 六、发射机构
 * ========================================================================== */

/* --- 摩擦轮(M3508 速度环, 单位 度/秒(转子)) --- */
/* ★ 这三档要实测! 现在是占位值。先给 30000(约 5000rpm 转子)试转。 */
#define FRICTION_TEST_APS (30000.0f)
/* ---------------- 摩擦轮初速反解 ----------------
 * 物理关系: v_bullet = eta * omega * R
 *   轮直接装在 M3508 转子上(无减速箱), 所以电机转速 = 轮转速
 *   例: v=20m/s, eta=0.90, R=0.035m(直径70mm)
 *       omega = 20/(0.9*0.035) = 634.9 rad/s = 6063 rpm = 36375 deg/s
 * 提示: 电流不能反解速度(取决于负载), 所以这里控的是转速;
 *       稳态电流可以用日志实测出来, 以后要前馈再把它填进 FRICTION_CURRENT_FF。
 */
#define FRICTION_BULLET_SPEED_MPS (20.0f) /* 目标弹丸初速 m/s(规则要求 15~25) */
#define FRICTION_WHEEL_RADIUS_M (0.035f)  /* 摩擦轮半径 m(直径 70mm) */
#define FRICTION_SLIP (0.90f)             /* 打滑/效率系数 0.85~0.95, 按实弹校准 */
#define FRICTION_TARGET_APS \
    (FRICTION_BULLET_SPEED_MPS / (FRICTION_SLIP * FRICTION_WHEEL_RADIUS_M) * 57.29578f)

/* 镜像方向: 轮子装反/电机线序不同就取 -1。
 * 约定(参考 mc-02 哨兵): 左轮取负, 右轮正常; 两轮对转往中间卷。 */
#define FRICTION_L_DIR (-1.0f)
#define FRICTION_R_DIR (-1.0f)

 /* 没标定时的兜底转速 */
#define FRICTION_SPEED_KP (5.0f)
#define FRICTION_SPEED_KI (1.0f)
#define FRICTION_SPEED_MAXOUT (12000.0f)
#define FRICTION_TEST_RPM (0.0f) /* >0: 台架测试用转速上限(rpm), 0=用初速反解值(6063rpm) */
#define FRICTION_RAMP_MS (500.0f)        /* 起转斜坡: 从 0 升到目标转速用多少 ms */
#define FRICTION_BRAKE_MS (400.0f)       /* 关闭时最多刹车多久(之后断输出) */
#define FRICTION_BRAKE_STOP_APS (500.0f) /* 转速低于此值就直接断输出 */

/* --- 拨弹盘(DM4310, 速度模式) --- */
#define LOADER_SLOTS (12)              /* 每圈几个槽(决定单发角度) */
#define LOADER_ANGLE_DEG (30.0f)          /* 单发推进角度 = 一个齿距(360/12), 不要多给, 否则相位累积 */
#define LOADER_ANGLE_RPM      (60.0f)     /* 兼容旧宏: 前段转速 */
#define LOADER_ANGLE_RPM_FAST (80.0f)     /* 前段推进转速(输出轴 rpm) */
#define LOADER_ANGLE_RPM_SLOW (10.0f)     /* 接近目标时的慢速, 减小惯性超调(单发打两发就是这个太快) */
#define LOADER_ANGLE_SLOW_ZONE_DEG (15.0f) /* 距目标多少度开始降速 */
#define LOADER_ANGLE_CORR_MAX (6.0f)      /* 单发补偿量上限(度), 防止异常时补过头 */
#define LOADER_ANGLE_STOP_MARGIN_DEG (1.0f) /* 提前多少度停, 用实测日志校准(=惯性超调量) */
#define LOADER_ANGLE_COOLDOWN_MS (250U)   /* 单发完成后冷却, 防止一次拨动连发两发 */
#define LOADER_ANGLE_TIMEOUT_MS (1000U)   /* 单发超时, 判定卡弹 */
#define LOADER_ONE_BULLET_DEG (360.0f / (float)LOADER_SLOTS)
#define LOADER_FIRE_RATE_HZ (10.0f)    /* 连发射频 */
#define LOADER_FIRE_RPM (LOADER_FIRE_RATE_HZ * 60.0f / (float)LOADER_SLOTS)
#define LOADER_SINGLE_RPM (60.0f)      /* 单发时的转速(输出轴) */
#define LOADER_ACC_RPM_S (600.0f)      /* 加速斜率 rpm/s */
#define LOADER_DIR (1.0f)              /* 装反了就改 -1 */

/* 相电流上限(写 0x03)。0.14 x 20A x 0.714 Nm/A ≈ 2.0 Nm 天花板 */
#define LOADER_OC (0.14f)

/* 卡弹判据: 要它转、转不动、扭矩还大 -> 持续 jam_ms 判卡弹 */
#define LOADER_JAM_MIN_RPM (30.0f) /* 低于此转速不判速度比卡弹(连发~12rpm), 靠扭矩保护 */
#define LOADER_JAM_SPEED_RATIO (0.35f)
#define LOADER_JAM_TQ (1.2f)
#define LOADER_JAM_MS (300)
#define LOADER_TQ_MAX (1.6f)
#define LOADER_TQ_HOLD_MS (100)
#define LOADER_GRACE_MS (500)  /* 刚使能后的宽限期, 不判卡弹 */
#define LOADER_RETRY_MAX (3)   /* 连续卡弹到达这个次数 -> 回拨大角度(不再失能) */
#define LOADER_JAM_CLEAR_MS (3000U) /* 正常转这么久就把"连续卡弹"计数清零 */
#define LOADER_BACKOFF_DEG (60.0f)  /* 连续卡弹后回拨的角度 */
#define LOADER_BACKOFF_RPM (30.0f)  /* 回拨转速(输出轴 rpm) */
#define LOADER_BACKOFF_TIMEOUT_MS (1500U) /* 回拨超时 */

/* ================= 发射热量管理(参考 2027 高校联盟赛规则 V1.0.0) =================
 * 结构: 底层状态机估计(数) + 上层前馈规划(算) + 硬限幅(兜底)
 * 默认: 步兵 17mm / 冷却优先   -> 上限 88, 冷却 24/s, 每发 10
 * 切爆发优先: 把 SHOOT_HEAT_COOLING_FIRST 改成 0 (上限 230, 冷却 14/s)
 */
#define RM_LOADER_HOLD_ENABLE (0)      /* 1: 不用遥控也让拨盘"使能并保持 0 转"(灯变绿, 不转动)
                                        * 用于接线/使能自检; 正常比赛改 0 */
#define LOADER_RX_AUTODETECT (0) /* 已改用多 ID 别名(同例程), 不再需要探测 */       /* 1: 收不到反馈时在 rx_id=0 / =LOADER_CAN_ID 之间自动探测
                                        * (达妙反馈帧 ID = Master ID, 出厂可能是 0 也可能是 1) */
#define RM_BENCH_SHOOT_ENABLE (0)      /* 台架调试: 1=不用遥控也能测发射机构(按键切换模式)
                                        * 上场前务必改回 0! 改 0 后回到"遥控 S1 上 + 拨轮"的控制 */
#define SHOOT_HEAT_BENCH_BYPASS (1)    /* 台架调试: 1=热量限幅旁路(连发按机械速率), 与摩擦轮无关
                                        * 上场前改回 0, 恢复 a/d 稳态射频与硬限幅 */

#define SHOOT_HEAT_ENABLE (1)          /* 1: 启用热量管理(状态机+前馈+限幅) */
#define SHOOT_HEAT_COOLING_FIRST (1)   /* 1: 冷却优先(88/24)  0: 爆发优先(230/14) */
#define SHOOT_HEAT_LIMIT_COOL (88.0f)  /* 冷却优先: 热量上限 */
#define SHOOT_HEAT_COOL_RPS_COOL (24.0f) /* 冷却优先: 冷却速率 /s */
#define SHOOT_HEAT_LIMIT_BURST (230.0f)  /* 爆发优先: 热量上限 */
#define SHOOT_HEAT_COOL_RPS_BURST (14.0f) /* 爆发优先: 冷却速率 /s */
#define SHOOT_HEAT_PER_SHOT (10.0f)    /* 每发消耗: 17mm=10, 42mm=100 */
#define SHOOT_HEAT_SETTLE_MS (100.0f)  /* 裁判系统结算周期 */
#define SHOOT_HEAT_MAX_RPS (10.0f)     /* 机械可支持的连发射频上限(发/s) */
#define SHOOT_HEAT_POLICY_STEADY (1)   /* 1: 稳态优先(射频≈a/d)  0: 允许吃满缓冲 */

/* 发弹确认: 摩擦轮电流阈值(A) + 持续时间; 需要实车标定 */
#define SHOOT_HEAT_DETECT_AMP (6.0f) /* 摩擦轮稳态电流也在 1~3A, 阈值要高于它; 实测后再校准 */
#define SHOOT_HEAT_DETECT_MS (20.0f)
#define SHOOT_HEAT_DETECT_REARM_RATIO (0.6f) /* 电流掉到阈值这个比例以下才重新武装 */
#define SHOOT_HEAT_FRICTION_READY_APS (12000.0f) /* 摩擦轮转速高于此值才算"已起转" */
#define SHOOT_HEAT_CONFIRM_TIMEOUT_MS (300.0f) /* 下令后这么久没确认 -> 按打了算(保守) */
#define SHOOT_HEAT_SOFT_RATIO (0.85f)  /* 只用于日志告警的软阈值 */
#define SHOOT_HEAT_HARD_RATIO (0.95f)  /* 只用于日志告警的硬阈值 */

/* VOFA+ 波形(UART1, JustFloat): 8 通道
 *   Qest / Q0 / 允许射频 / 指令射频 / 指令rpm / 实测rpm / 已确认发数 / 状态位
 * UART1 默认 115200, 一帧 36B 约 3ms, 所以 100Hz/4 = 25Hz 发送 */
#define SHOOT_HEAT_VOFA_ENABLE (1)
#define SHOOT_HEAT_VOFA_UART BOARD_USART1
#define SHOOT_HEAT_VOFA_DIV (4)

/* 自恢复: 停 -> 反转退弹 -> 再正转 */
#define LOADER_REVERSE_RPM (25.0f)
#define LOADER_REVERSE_MS (400)
#define LOADER_SETTLE_MS (150)

/* 上电时序: 等电机自检 / 切模式重试间隔 */
#define LOADER_BOOT_WAIT_MS (800)
#define LOADER_ENABLE_RETRY_MS (500)

/* ==========================================================================
 * 七、遥控映射(DR16 / DT7)
 *
 *   S1(左开关): 上 = 正常工作(云台+发射)   中 = 只调云台(发射禁用)   下 = 急停
 *   S2(右开关): 上 = 小陀螺               中 = 不跟随(可平移)        下 = 跟随云台
 *   右摇杆    : 上下 = 前进/后退, 左右 = 左右平移
 *   左摇杆    : 左右 = Yaw 转速, 上下 = Pitch 转速
 *   左拨轮    : 全部抬起 = 摩擦轮开; 推到底 = 连发
 * ========================================================================== */
#define RC_VX_MAX (2.0f)   /* m/s */
#define RC_VY_MAX (2.0f)   /* m/s */
#define RC_WZ_MAX (6.0f)   /* rad/s 小陀螺角速度 */
#define RC_DIAL_FRICTION_ON (-150) /* 拨轮低于这个开摩擦轮 */
#define RC_DIAL_FIRE (-550)        /* 拨轮低于这个连发 */

/* 方向修正: 实测发现"推前进却后退/摇杆左右反了"就把对应的改成 -1.0f */
#define RC_VX_SIGN (1.0f)
#define RC_VY_SIGN (1.0f)
#define RC_YAW_SIGN (1.0f)
#define RC_PITCH_SIGN (1.0f)

#endif /* RM_CONFIG_H */
