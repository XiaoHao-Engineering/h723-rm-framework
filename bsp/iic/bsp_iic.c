/**
 * @file bsp_iic.c
 * @brief I2C 板级支持包(H7 版)
 *
 * @note MC02 上可用的 I2C:
 *        - I2C2: PB10(SCL) / PB11(SDA), 位于 LCD 扩展接口
 *        - I2C3: PA8/PB8 与 DCMI 接口共用, 默认不启用
 *       本文件不依赖 CubeMX 是否配置了 I2C, 使用哪个 I2C 由 module 传进来的句柄决定.
 *       若要使用, 请先在 CubeMX 中打开对应 I2C 并在 module 中注册.
 */

#include "bsp_iic.h"
#include "bsp_log.h"

#include <stdlib.h>
#include <string.h>

static uint8_t idx;
static IICInstance *iic_instance[IIC_MX_SLAVE_CNT] = {NULL};

IICInstance *IICRegister(IIC_Init_Config_s *conf)
{
    if (idx >= IIC_MX_SLAVE_CNT)
        while (1)
            LOGERROR("[bsp_iic] IIC instance exceeded max num!");

    IICInstance *iic = (IICInstance *)malloc(sizeof(IICInstance));
    memset(iic, 0, sizeof(IICInstance));

    iic->handle = conf->handle;
    iic->dev_address = conf->dev_address;
    iic->work_mode = conf->work_mode;
    iic->callback = conf->callback;
    iic->id = conf->id;

    iic_instance[idx++] = iic;
    return iic;
}

void IICSetMode(IICInstance *iic, IIC_Work_Mode_e mode)
{
    iic->work_mode = mode;
}

static uint32_t IICXferOption(IIC_Seq_Mode_e mode)
{
    return (mode == IIC_SEQ_HOLDON) ? I2C_FIRST_FRAME : I2C_FIRST_AND_LAST_FRAME;
}

void IICTransmit(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e mode)
{
    uint16_t addr = (uint16_t)(iic->dev_address << 1);

    switch (iic->work_mode)
    {
    case IIC_BLOCK_MODE:
        HAL_I2C_Master_Transmit(iic->handle, addr, data, size, 10);
        break;
    case IIC_IT_MODE:
        HAL_I2C_Master_Seq_Transmit_IT(iic->handle, addr, data, size, IICXferOption(mode));
        break;
    case IIC_DMA_MODE:
        HAL_I2C_Master_Seq_Transmit_DMA(iic->handle, addr, data, size, IICXferOption(mode));
        break;
    default:
        LOGERROR("[bsp_iic] illegal iic work mode");
        break;
    }
}

void IICReceive(IICInstance *iic, uint8_t *data, uint16_t size, IIC_Seq_Mode_e mode)
{
    uint16_t addr = (uint16_t)(iic->dev_address << 1);

    iic->rx_buffer = data;
    iic->rx_len = (uint8_t)size;

    switch (iic->work_mode)
    {
    case IIC_BLOCK_MODE:
        HAL_I2C_Master_Receive(iic->handle, addr, data, size, 10);
        break;
    case IIC_IT_MODE:
        HAL_I2C_Master_Seq_Receive_IT(iic->handle, addr, data, size, IICXferOption(mode));
        break;
    case IIC_DMA_MODE:
        HAL_I2C_Master_Seq_Receive_DMA(iic->handle, addr, data, size, IICXferOption(mode));
        break;
    default:
        LOGERROR("[bsp_iic] illegal iic work mode");
        break;
    }
}

void IICAccessMem(IICInstance *iic, uint16_t mem_addr, uint8_t *data, uint16_t size, IIC_Mem_Mode_e mode, uint8_t mem8bit_flag)
{
    uint16_t addr = (uint16_t)(iic->dev_address << 1);
    uint16_t mem_size = mem8bit_flag ? I2C_MEMADD_SIZE_8BIT : I2C_MEMADD_SIZE_16BIT;

    if (mode == IIC_WRITE_MEM)
        HAL_I2C_Mem_Write(iic->handle, addr, mem_addr, mem_size, data, size, 10);
    else
        HAL_I2C_Mem_Read(iic->handle, addr, mem_addr, mem_size, data, size, 10);
}

/* -------------------- HAL 回调分发 -------------------- */

static void IICDispatch(I2C_HandleTypeDef *handle)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (iic_instance[i]->handle == handle)
        {
            if (iic_instance[i]->callback != NULL)
                iic_instance[i]->callback(iic_instance[i]);
            return;
        }
    }
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    IICDispatch(hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    IICDispatch(hi2c);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    IICDispatch(hi2c);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    IICDispatch(hi2c);
}
