/**
 * @file bsp_spi.c
 * @brief SPI 板级支持包(H7 版)
 */

#include "bsp_spi.h"
#include "bsp_log.h"

#include <stdlib.h>
#include <string.h>

/* 所有的spi instance保存于此,用于callback时判断中断来源*/
static SPIInstance *spi_instance[SPI_MX_REGISTER_CNT] = {NULL};
static uint8_t idx = 0; // 配合中断以及初始化

SPIInstance *SPIRegister(SPI_Init_Config_s *conf)
{
    if (idx >= SPI_MX_REGISTER_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_spi] SPI instance exceeded max num!");

    SPIInstance *instance = (SPIInstance *)malloc(sizeof(SPIInstance));
    memset(instance, 0, sizeof(SPIInstance));

    instance->spi_handle = conf->spi_handle;
    instance->GPIOx = conf->GPIOx;
    instance->cs_pin = conf->cs_pin;
    instance->spi_work_mode = conf->spi_work_mode;
    instance->callback = conf->callback;
    instance->id = conf->id;

    spi_instance[idx++] = instance;
    return instance;
}

void SPITransmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len)
{
    // 拉低片选,开始传输(选中从机)
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode)
    {
    case SPI_DMA_MODE:
        HAL_SPI_Transmit_DMA(spi_ins->spi_handle, ptr_data, len);
        break;
    case SPI_IT_MODE:
        HAL_SPI_Transmit_IT(spi_ins->spi_handle, ptr_data, len);
        break;
    case SPI_BLOCK_MODE:
        HAL_SPI_Transmit(spi_ins->spi_handle, ptr_data, len, 1000);
        // 阻塞模式不会调用回调函数,传输完成后直接拉高片选结束
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        break;
    default:
        while (1)
            ; // error mode! 请查看是否正确设置模式，或出现指针越界
    }
}

void SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len)
{
    // 用于稍后回调使用
    spi_ins->rx_size = len;
    spi_ins->rx_buffer = ptr_data;
    // 拉低片选,开始传输
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode)
    {
    case SPI_DMA_MODE:
        HAL_SPI_Receive_DMA(spi_ins->spi_handle, ptr_data, len);
        break;
    case SPI_IT_MODE:
        HAL_SPI_Receive_IT(spi_ins->spi_handle, ptr_data, len);
        break;
    case SPI_BLOCK_MODE:
        HAL_SPI_Receive(spi_ins->spi_handle, ptr_data, len, 1000);
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        break;
    default:
        while (1)
            ; // error mode!
    }
}

void SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len)
{
    // 请保证ptr_data_rx在回调函数被调用之前仍然在作用域内
    spi_ins->rx_size = len;
    spi_ins->rx_buffer = ptr_data_rx;
    HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_RESET);
    switch (spi_ins->spi_work_mode)
    {
    case SPI_DMA_MODE:
        HAL_SPI_TransmitReceive_DMA(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len);
        break;
    case SPI_IT_MODE:
        HAL_SPI_TransmitReceive_IT(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len);
        break;
    case SPI_BLOCK_MODE:
        HAL_SPI_TransmitReceive(spi_ins->spi_handle, ptr_data_tx, ptr_data_rx, len, 1000);
        HAL_GPIO_WritePin(spi_ins->GPIOx, spi_ins->cs_pin, GPIO_PIN_SET);
        break;
    default:
        while (1)
            ; // error mode!
    }
}

void SPISetMode(SPIInstance *spi_ins, SPI_TXRX_MODE_e spi_mode)
{
    if (spi_mode != SPI_DMA_MODE && spi_mode != SPI_IT_MODE && spi_mode != SPI_BLOCK_MODE)
        while (1)
            ; // error mode!

    if (spi_ins->spi_work_mode != spi_mode)
        spi_ins->spi_work_mode = spi_mode;
}

/**
 * @brief SPI 接收完成回调: 拉高片选并调用模块回调
 */
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
    for (uint8_t i = 0; i < idx; i++)
    {
        if (spi_instance[i]->spi_handle == hspi &&
            HAL_GPIO_ReadPin(spi_instance[i]->GPIOx, spi_instance[i]->cs_pin) == GPIO_PIN_RESET)
        {
            HAL_GPIO_WritePin(spi_instance[i]->GPIOx, spi_instance[i]->cs_pin, GPIO_PIN_SET);
            if (spi_instance[i]->callback != NULL)
                spi_instance[i]->callback(spi_instance[i]);
            return;
        }
    }
}

/**
 * @brief 和RxCpltCallback共用解析即可
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
    HAL_SPI_RxCpltCallback(hspi);
}
