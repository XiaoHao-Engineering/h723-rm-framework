/**
 * @file bsp_usb.c
 * @brief USB 虚拟串口板级支持包(H7 版, USB_OTG_HS 工作在 FS 模式)
 */

#include "bsp_usb.h"
#include "bsp_log.h"

static uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里, buffer size 见 APP_RX_DATA_SIZE

uint8_t *USBInit(USB_Init_Config_s usb_conf)
{
    /* USB 的软件复位(模拟拔插)在 usbd_conf.c 的 HAL_PCD_MspInit() 中完成 */
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(usb_conf.tx_cbk, usb_conf.rx_cbk);

    if (bsp_usb_rx_buffer != NULL)
        LOGINFO("[bsp_usb] usb cdc init success");
    return bsp_usb_rx_buffer;
}

void USBTransmit(uint8_t *buffer, uint16_t len)
{
    CDC_Transmit_HS(buffer, len);
}
