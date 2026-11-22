#ifndef BSP_IIC_H
#define BSP_IIC_H

#include "main.h"
#include "stdint.h"

#define IIC_MX_SLAVE_CNT 8 // 最大从机实例数目,根据需要修改

/* i2c 工作模式枚举 */
typedef enum
{
    IIC_BLOCK_MODE = 0, // 阻塞模式
    IIC_IT_MODE,        // 中断模式
    IIC_DMA_MODE,       // DMA模式
} IIC_Work_Mode_e;

/* I2C MEM工作模式枚举,这两种方法都是阻塞 */
typedef enum
{
    IIC_READ_MEM = 0, // 读取从机内部的寄存器or内存
    IIC_WRITE_MEM,    // 写入从机内部的寄存器or内存
} IIC_Mem_Mode_e;

/* Seq传输工作模式枚举,注意HOLD_ON要在IT或DMA下使用 */
typedef enum
{
    IIC_SEQ_RELEASE = 0, // 完成传输后释放总线占有权,这是默认的传输方式
    IIC_SEQ_HOLDON,      // 保持总线占有权不释放,只支持IT和DMA模式
} IIC_Seq_Mode_e;

/* i2c实例 */
typedef struct iic_temp_s
{
    I2C_HandleTypeDef *handle; // i2c handle
    uint8_t dev_address;       // 7位地址(不含读写位)

    IIC_Work_Mode_e work_mode;             // 工作模式
    uint8_t *rx_buffer;                    // 接收缓冲区指针
    uint8_t rx_len;                        // 接收长度
    void (*callback)(struct iic_temp_s *); // 接收完成后的回调函数

    void *id; // 用于标识i2c instance
} IICInstance;

/* I2C 初始化结构体配置 */
typedef struct
{
    I2C_HandleTypeDef *handle;       // i2c handle
    uint8_t dev_address;             // 7位地址(不需要左移)
    IIC_Work_Mode_e work_mode;       // 工作模式
    void (*callback)(IICInstance *); // 接收完成后的回调函数
    void *id;                        // 用于标识i2c instance
} IIC_Init_Config_s;

IICInstance *IICRegister(IIC_Init_Config_s *conf);
void IICSetMode(IICInstance *iic, IIC_Work_Mode_e mode);
void IICTransmit(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e mode);
void IICReceive(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e mode);
void IICAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, IIC_Mem_Mode_e mode, uint8_t mem8bit_flag);

#endif /* BSP_IIC_H */
