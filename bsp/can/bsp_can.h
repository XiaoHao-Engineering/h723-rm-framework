#ifndef BSP_CAN_H
#define BSP_CAN_H

#ifndef CAN_LOOPBACK_SELFTEST
#define CAN_LOOPBACK_SELFTEST 0 /* 1: 上电做一次 CAN 控制器内部回环自检 */
#endif

#ifndef CAN_ID_CENSUS
#define CAN_ID_CENSUS 1 /* 1: 统计总线上每个 ID 的帧率(临时诊断) */
#endif


#include <stdint.h>
#include "fdcan.h"

/* 最多能够支持的 CAN 设备数(取决于总线负载, 通常每个实例对应一台电机/一个设备) */
#define CAN_MX_REGISTER_CNT 24
/* 板上 FDCAN 的个数: MC02 有 FDCAN1/2/3 */
#define DEVICE_CAN_CNT 3
/* 单帧最大数据长度: 经典 CAN 为 8, CANFD 为 64 */
#define CAN_MAX_DATA_LEN 64

/* 帧类型: 经典 CAN 或 CANFD */
typedef enum
{
    CAN_FRAME_CLASSIC = 0,
    CAN_FRAME_FD,
} CAN_FrameType_e;

/* can instance typedef, every module registered to CAN should have this variable */
#pragma pack(1)
typedef struct _
{
    FDCAN_HandleTypeDef *can_handle; // FDCAN 句柄, 如 &hfdcan1
    FDCAN_TxHeaderTypeDef txconf;    // 发送报文配置(每次发送前按 tx_len 更新)
    uint32_t tx_id;                  // 发送id
    uint32_t rx_id;                  // 接收id
    uint8_t tx_buff[CAN_MAX_DATA_LEN]; // 发送缓存, 默认长度8, 可用 CANSetDLC() 或 CANSetFD() 修改
    uint8_t rx_buff[CAN_MAX_DATA_LEN]; // 接收缓存
    uint8_t rx_len;                    // 最近一次接收到的数据长度
    uint8_t tx_len;                    // 本次发送的数据长度
    uint8_t is_fd;                     // 1: 以 CANFD 帧发送
    uint8_t brs;                       // 1: 开启比特率切换(CANFD BRS)
    /* 接收的回调函数, 用于解析接收到的数据 */
    void (*can_module_callback)(struct _ *);
    void *id; // 使用该 can 实例的模块指针(父子关系)
} CANInstance;
#pragma pack()

/* CAN 实例初始化结构体, 将此结构体指针传入注册函数 */
typedef struct
{
    FDCAN_HandleTypeDef *can_handle;            // FDCAN 句柄
    uint32_t tx_id;                             // 发送id
    uint32_t rx_id;                             // 接收id
    void (*can_module_callback)(CANInstance *); // 处理接收数据的回调函数
    void *id;                                   // 拥有can实例的模块地址
    uint32_t nominal_bitrate;                   // 仲裁段波特率, 0 表示默认 1000000(1Mbps)
    uint32_t data_bitrate;                      // 数据段波特率(FD), 0 表示默认 5000000(5Mbps)
    uint8_t is_fd;                              // 1: 使用 CANFD 帧通信
    uint8_t brs;                                // 1: CANFD 数据段提速(BRS)
} CAN_Init_Config_s;

/**
 * @brief 注册(初始化)一个can实例, 需要传入初始化配置的指针
 *
 * @note 同一条总线上的第一个实例会按需要重新配置该 FDCAN 的位时序/缓冲区,
 *       因此同一总线上的所有设备应当使用相同的波特率与帧类型.
 *
 * @param config init config
 * @return CANInstance* can instance owned by module
 */
CANInstance *CANRegister(CAN_Init_Config_s *config);

/**
 * @brief 修改 CAN 发送报文的数据帧长度(经典CAN最大8, CANFD最大64)
 */
void CANSetDLC(CANInstance *_instance, uint8_t length);

/**
 * @brief 设置该实例发送时使用的帧类型
 *
 * @param is_fd 1: CANFD, 0: 经典 CAN
 * @param brs   仅 is_fd=1 时有效, 1 表示开启数据段提速
 */
void CANSetFrameType(CANInstance *_instance, uint8_t is_fd, uint8_t brs);

/**
 * @brief 通过 can 实例发送消息; 发送前需要向实例的 tx_buff 写入数据
 *
 * @attention 超时时间不应该超过调用此函数的任务的周期, 否则会导致任务阻塞
 *
 * @param timeout 超时时间, 单位为 ms
 */
uint8_t CANTransmit(CANInstance *_instance, float timeout);

/**
 * @brief CAN 控制器内部回环自检(不外发, 不会驱动电机)
 * @return 1=回环成功(MCU 侧正常) 0=失败
 */
uint8_t BSPCAN_InternalLoopbackTest(FDCAN_HandleTypeDef *handle, uint32_t test_id);


#if CAN_ID_CENSUS
extern volatile uint16_t can_census_id[16];
extern volatile uint32_t can_census_cnt[16];
extern volatile uint8_t can_census_num;
#endif

#endif /* BSP_CAN_H */
