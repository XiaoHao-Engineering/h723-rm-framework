#ifndef CHASSIS_OMNI_H
#define CHASSIS_OMNI_H

#include "dji_motor.h"
#include "bsp_board.h"

/* 机械参数(m) */
#define OMNI_WHEELBASE (0.300f) /* 前轮中心<->后轮中心 */
#define OMNI_TRACK     (0.300f) /* 左轮中心<->右轮中心 */
#define OMNI_WHEEL_R   (0.076f)
/* 减速比: 新全向轮 268/17 ≈ 15.76 (旧 M3508 P19 为 3591/187 ≈ 19.2) */
#define OMNI_RATIO     (268.0f / 17.0f)

/* 轮序 */
#define OMNI_FRONT (0)
#define OMNI_REAR  (1)
#define OMNI_LEFT  (2)
#define OMNI_RIGHT (3)
#define OMNI_WHEEL_CNT (4)

/* 电机 ID 与转向 */
#define OMNI_CAN_BUS   BOARD_CAN1
#define OMNI_ID_FRONT  (1)
#define OMNI_ID_REAR   (2)
#define OMNI_ID_LEFT   (3)
#define OMNI_ID_RIGHT  (4)

#define OMNI_REV_FRONT (1)
#define OMNI_REV_REAR  (1)
#define OMNI_REV_LEFT  (1)
#define OMNI_REV_RIGHT (1)

/* 速度上限 */
#define OMNI_MAX_VX (2.0f)
#define OMNI_MAX_VY (2.0f)
#define OMNI_MAX_W  (5.0f)

/* 轮速环 */
#define OMNI_SPEED_KP       (15.0f)
#define OMNI_SPEED_KI       (0.15f)
#define OMNI_SPEED_KD       (0.0f)
#define OMNI_SPEED_MAX_OUT  (16384.0f)
#define OMNI_SPEED_MAX_IOUT (4000.0f)

/* 总功率限幅(W), 0=关闭 */
#define OMNI_POWER_MAX (200.0f)
#define OMNI_VOLTAGE   (24.0f)

/* 遥控映射: 0=右横 1=右竖 2=左横 3=左竖 */
#define OMNI_RC_ENABLE (0)
#define OMNI_RC_VX_CH  (3)
#define OMNI_RC_VY_CH  (2)
#define OMNI_RC_W_CH   (0)
#define OMNI_RC_DEAD   (10)

void ChassisOmniInit(void);
void ChassisOmniSetVelocity(float vx, float vy, float omega);
void ChassisOmniGetVelocity(float *vx, float *vy, float *omega);
void ChassisOmniTask(void);
void ChassisOmniStop(void);
uint8_t ChassisOmniIsOnline(void);

#endif /* CHASSIS_OMNI_H */
