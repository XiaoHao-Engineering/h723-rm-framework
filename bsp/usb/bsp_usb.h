/**
 * @file bsp_usb.h
 * @brief USB 虚拟串口(VCP)板级支持包
 *
 * @attention bsp_usb 依赖 usbd_cdc_if.c 中 "USER CODE" 区的两个函数:
 *            CDCInitRxbufferNcallback() / CDC_Transmit_HS(),
 *            即使以后用 CubeMX 重新生成 USB 代码, 这两个函数也会保留.
 */
#pragma once

#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"

typedef struct
{
    USBCallback tx_cbk; // 发送完成回调, 参数为本次发送的字节数, 可为 NULL
    USBCallback rx_cbk; // 接收回调, 参数为本次接收的字节数, 可为 NULL
} USB_Init_Config_s;

/**
 * @brief 初始化 USB 虚拟串口(会重新枚举设备), 返回接收缓冲区首地址
 * @note  接收缓冲区大小见 APP_RX_DATA_SIZE(默认 2048)
 */
uint8_t *USBInit(USB_Init_Config_s usb_conf);

/**
 * @brief 通过 USB 发送数据
 * @note  全速模式下单个数据包最大 64 字节, 一次发送超过 64 字节会被拆包, 上位机需要按长度组包
 */
void USBTransmit(uint8_t *buffer, uint16_t len);
