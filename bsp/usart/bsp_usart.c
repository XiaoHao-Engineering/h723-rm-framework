/**
 * @file bsp_usart.c
 * @brief 串口板级支持包(H7 版): DMA + 空闲中断接收, 阻塞/中断/DMA 发送
 *
 * @note H7 上使用 DMA 的缓冲区必须放在 DMA 可访问的内存里(不能是 DTCM),
 *       本工程的 FreeRTOS 堆位于 AXI SRAM(见链接脚本的 .dma_buffer 段与 MPU 配置),
 *       因此 malloc 出来的 USARTInstance 中的 recv_buff 可以被 DMA 直接使用.
 */

#include "bsp_usart.h"
#include "bsp_board.h"
#include "bsp_log.h"

#include <stdlib.h>
#include <string.h>

/* usart服务实例,所有注册了usart的模块信息会被保存在这里 */
static uint8_t idx;
static USARTInstance *usart_instance[DEVICE_USART_CNT] = {NULL};

/**
 * @brief 串口实例的静态存储池
 *
 * @note H7 的 DTCM 不能被 DMA 访问, 而 malloc 默认从 DTCM 的 newlib 堆分配,
 *       所以这里用固定数组 + DMA_BUFFER 宏把实例(含 recv_buff)放到 AXI SRAM.
 */
static USARTInstance usart_instance_pool[DEVICE_USART_CNT] DMA_BUFFER;

/**
 * @brief 启动(或重启)某个实例的 DMA 空闲中断接收
 */
static void USARTStartReceive(USARTInstance *instance)
{
    HAL_UARTEx_ReceiveToIdle_DMA(instance->usart_handle, instance->recv_buff, instance->recv_buff_size);

    /* HAL 在 DMA 半传输/传输完成/IDLE 时都会调用 HAL_UARTEx_RxEventCallback,
     * 我们只需要"传输完成"和"IDLE"两种, 所以关掉半传输中断 */
    if (instance->usart_handle->hdmarx != NULL)
        __HAL_DMA_DISABLE_IT(instance->usart_handle->hdmarx, DMA_IT_HT);
}

void USARTServiceInit(USARTInstance *_instance)
{
    USARTStartReceive(_instance);
}

USARTInstance *USARTRegister(USART_Init_Config_s *init_config)
{
    if (idx >= DEVICE_USART_CNT) // 超过最大实例数
        while (1)
            LOGERROR("[bsp_usart] USART exceed max instance count!");

    for (uint8_t i = 0; i < idx; i++) // 检查是否已经注册过
        if (usart_instance[i]->usart_handle == init_config->usart_handle)
            while (1)
                LOGERROR("[bsp_usart] USART instance already registered!");

    USARTInstance *instance = &usart_instance_pool[idx];
    memset(instance, 0, sizeof(USARTInstance));

    instance->usart_handle = init_config->usart_handle;
    instance->recv_buff_size = init_config->recv_buff_size;
    instance->module_callback = init_config->module_callback;

    usart_instance[idx++] = instance;
    USARTStartReceive(instance);
    return instance;
}

void USARTSend(USARTInstance *_instance, uint8_t *send_buf, uint16_t send_size, USART_TRANSFER_MODE mode)
{
    switch (mode)
    {
    case USART_TRANSFER_BLOCKING:
        HAL_UART_Transmit(_instance->usart_handle, send_buf, send_size, 100);
        break;
    case USART_TRANSFER_IT:
        HAL_UART_Transmit_IT(_instance->usart_handle, send_buf, send_size);
        break;
    case USART_TRANSFER_DMA:
        HAL_UART_Transmit_DMA(_instance->usart_handle, send_buf, send_size);
        break;
    default:
        while (1)
            ; // illegal mode! 检查定义instance的代码上下文,可能出现指针越界
    }
}

uint8_t USARTIsReady(USARTInstance *_instance)
{
    return (_instance->usart_handle->gState == HAL_UART_STATE_READY) ? 1 : 0;
}

/**
 * @brief 失效指定地址范围的 D-Cache(按 32 字节行)
 *
 * @note 不依赖 __DCACHE_PRESENT 宏(本工程该宏为 0, CMSIS 的 SCB_InvalidateDCache_by_Addr
 *       不会被编译进来), 直接写 DCIMVAC 维护寄存器。
 */
static void USART_RxCacheInvalidate(void *addr, uint32_t size)
{
    uint32_t start, end, a;

    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U) /* D-Cache 没开就不用管 */
        return;
    if (size == 0U)
        return;

    start = ((uint32_t)addr) & ~31U;
    end = ((uint32_t)addr + size + 31U) & ~31U;

    for (a = start; a < end; a += 32U)
        SCB->DCIMVAC = a; /* 失效该缓存行(丢弃脏数据, 下次读回内存) */

    __DSB();
    __ISB();
}

/**
 * @brief 空闲中断/DMA 传输完成时触发, 分发到对应模块的回调
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            if (Size > usart_instance[i]->recv_buff_size)
                Size = usart_instance[i]->recv_buff_size;
            usart_instance[i]->recv_len = Size;

            /* ★ H7 开了 D-Cache: DMA 写进内存的数据, CPU 可能读到缓存里的旧值(例如 memset 的 0)。
             * 解析前必须按 32 字节缓存行失效这段地址。
             * 注意: 本工程 CMSIS 的 __DCACHE_PRESENT 是 0, SCB_InvalidateDCache_by_Addr()
             * 根本没被编译进去, 所以这里直接写 DCIMVAC 维护寄存器。 */
            USART_RxCacheInvalidate(usart_instance[i]->recv_buff, Size);

            if (usart_instance[i]->module_callback != NULL)
                usart_instance[i]->module_callback();

            /* ★ 不要 memset!
             * buffer 在可缓存的 AXI SRAM 里, memset 会把缓存行写成"脏"的,
             * 缓存回写时会把 DMA 刚写进来的数据覆盖成 0(这就是之前一直读到 0 的原因)。
             * 解析只用 Size 个字节, 不需要清零。 */
            USARTStartReceive(usart_instance[i]);
            return;
        }
    }
}

/**
 * @brief 串口出现错误(奇偶校验/溢出/帧错误)时重新启动接收
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (huart == usart_instance[i]->usart_handle)
        {
            HAL_UART_AbortReceive(usart_instance[i]->usart_handle);
            USARTStartReceive(usart_instance[i]);
            LOGWARNING("[bsp_usart] USART error callback triggered, instance idx [%d]", i);
            return;
        }
    }
}
