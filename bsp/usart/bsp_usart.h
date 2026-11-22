#ifndef BSP_USART_H
#define BSP_USART_H

#include <stdint.h>
#include "main.h"

/* MC02 上被 bsp 使用的串口个数: USART1/2/3, UART5(SBUS), UART7, USART10 */
#define DEVICE_USART_CNT 6
/* 如果协议需要更大的 buff, 请修改这里 */
#define USART_RXBUFF_LIMIT 256

/* 模块回调函数, 用于解析协议(参数为 void, 模块内部通过自己的实例取数据) */
typedef void (*usart_module_callback)(void);

/* 发送模式枚举 */
typedef enum
{
    USART_TRANSFER_NONE = 0,
    USART_TRANSFER_BLOCKING,
    USART_TRANSFER_IT,
    USART_TRANSFER_DMA,
} USART_TRANSFER_MODE;

/**
 * @brief 串口实例结构体, 每个 module 都要包含一个实例
 *
 * @note 接收采用 "DMA + 空闲中断" 的方式, 一帧数据到来后触发 module_callback.
 *       recv_buff 建议由 module 按协议最大长度使用(小于等于 recv_buff_size).
 */
typedef struct
{
    uint8_t recv_buff[USART_RXBUFF_LIMIT]; // 预先定义的最大buff大小
    uint16_t recv_buff_size;               // 模块接收一包数据的大小
    uint16_t recv_len;                     // 最近一次接收到的长度(由空闲中断回调更新)
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USARTInstance;

/* usart 初始化配置结构体 */
typedef struct
{
    uint16_t recv_buff_size;               // 模块接收一包数据的大小
    UART_HandleTypeDef *usart_handle;      // 实例对应的usart_handle
    usart_module_callback module_callback; // 解析收到的数据的回调函数
} USART_Init_Config_s;

/**
 * @brief 注册一个串口实例(会立刻开启 DMA + 空闲中断接收)
 *
 * @param init_config 传入串口初始化结构体
 */
USARTInstance *USARTRegister(USART_Init_Config_s *init_config);

/**
 * @brief 重新启动串口接收服务, 一般用于 lost callback(使用串口的模块 daemon 会调用)
 */
void USARTServiceInit(USARTInstance *_instance);

/**
 * @brief 通过串口发送一帧数据
 *
 * @note 在短时间内连续调用此接口, 若采用 IT/DMA 会导致上一次的发送未完成而新的发送取消.
 *       若希望连续使用 DMA/IT 进行发送, 请配合 USARTIsReady() 使用.
 */
void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode);

/**
 * @brief 判断串口是否准备好, 用于连续或异步的 IT/DMA 发送
 *
 * @return uint8_t 1: ready, 0: busy
 */
uint8_t USARTIsReady(USARTInstance *_instance);

#endif /* BSP_USART_H */
