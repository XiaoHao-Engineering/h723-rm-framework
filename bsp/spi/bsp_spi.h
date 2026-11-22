#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "spi.h"
#include "stdint.h"
#include "gpio.h"

/* 最多允许注册的 SPI 从机实例数(一条总线可以挂多个从机, 用 CS 区分) */
#define SPI_MX_REGISTER_CNT 8

/* spi transmit recv mode enumerate*/
typedef enum
{
    SPI_BLOCK_MODE = 0, // 默认使用阻塞模式
    SPI_IT_MODE,
    SPI_DMA_MODE,
} SPI_TXRX_MODE_e;

/* SPI实例结构体定义 */
typedef struct spi_ins_temp
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    SPI_TXRX_MODE_e spi_work_mode; // 传输工作模式
    uint8_t rx_size;               // 本次接收的数据长度
    uint8_t *rx_buffer;            // 本次接收的数据缓冲区

    void (*callback)(struct spi_ins_temp *); // 接收回调函数
    void *id;                                // 模块指针
} SPIInstance;

/* 接收回调函数定义,包含SPI的module按照此格式构建回调函数 */
typedef void (*spi_rx_callback)(SPIInstance *);

/* SPI初始化配置,其实基本和SPIIstance一模一样,为了代码风格统一因此再次定义 */
typedef struct
{
    SPI_HandleTypeDef *spi_handle; // SPI外设handle
    GPIO_TypeDef *GPIOx;           // 片选信号对应的GPIO,如GPIOA,GPIOB等等
    uint16_t cs_pin;               // 片选信号对应的引脚号,GPIO_PIN_1,GPIO_PIN_2等等

    SPI_TXRX_MODE_e spi_work_mode; // 传输工作模式

    spi_rx_callback callback; // 接收回调函数
    void *id;                 // 模块指针
} SPI_Init_Config_s;

/**
 * @brief 注册一个spi instance
 */
SPIInstance *SPIRegister(SPI_Init_Config_s *conf);

/**
 * @brief 通过spi向对应从机发送数据(会自动拉低/拉高片选)
 */
void SPITransmit(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len);

/**
 * @brief 通过spi从从机获取数据
 * @attention 请保证 ptr_data 在回调函数被调用之前仍然在作用域内
 */
void SPIRecv(SPIInstance *spi_ins, uint8_t *ptr_data, uint8_t len);

/**
 * @brief 通过spi利用移位寄存器同时收发数据
 */
void SPITransRecv(SPIInstance *spi_ins, uint8_t *ptr_data_rx, uint8_t *ptr_data_tx, uint8_t len);

/**
 * @brief 设定spi收发的工作模式
 */
void SPISetMode(SPIInstance *spi_ins, SPI_TXRX_MODE_e spi_mode);

#endif /* BSP_SPI_H */
