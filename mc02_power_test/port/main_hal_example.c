/*
 * STM32H7 (达妙 MC02) 移植示例
 * 对应 CubeMX: FDCAN1(PD0/PD1) + USART1(PA9/PA10, 921600) + I2C1 + TIM6(1kHz)
 * 把本文件里的 if 0 块换成你自己的句柄名即可。
 */
#include "main.h"
#include "power_test.h"
#include "c620.h"
#include "ina226.h"

#include <string.h>

extern FDCAN_HandleTypeDef hfdcan1;
extern UART_HandleTypeDef  huart1;
extern I2C_HandleTypeDef   hi2c1;
extern TIM_HandleTypeDef   htim6;

#define INA226_ADDR   0x40
#define INA_R_MOHM    2.0f      /* 看你的采样电阻：0.002Ω = 2mΩ */

/* ---------------------------------------------------------------- FDCAN */
static void can_start(void)
{
    FDCAN_FilterTypeDef f = {0};
    f.IdType       = FDCAN_STANDARD_ID;
    f.FilterIndex  = 0;
    f.FilterType   = FDCAN_FILTER_MASK;
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID1    = 0x000;
    f.FilterID2    = 0x000;              /* 全收 */
    HAL_FDCAN_ConfigFilter(&hfdcan1, &f);
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_ACCEPT_IN_RX_FIFO0,
                                 FDCAN_FILTER_REMOTE,
                                 FDCAN_FILTER_REMOTE);
    HAL_FDCAN_Start(&hfdcan1);
    HAL_FDCAN_ActivateNotification(&hfdcan1,
                                   FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void pt_port_can_send(uint32_t id, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef tx = {0};
    tx.Identifier          = id;
    tx.IdType              = FDCAN_STANDARD_ID;
    tx.TxFrameType         = FDCAN_DATA_FRAME;
    tx.DataLength          = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch       = FDCAN_BRS_OFF;
    tx.FDFormat            = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker       = 0;
    HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx, (uint8_t *)data);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *h, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx;
    uint8_t d[8];
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;
    if (HAL_FDCAN_GetRxMessage(h, FDCAN_RX_FIFO0, &rx, d) != HAL_OK) return;
    if (rx.IdType != FDCAN_STANDARD_ID) return;
    pt_on_can_rx(rx.Identifier, d, 8);
}

/* ---------------------------------------------------------------- 时钟/日志 */
uint32_t pt_port_now_ms(void)
{
    return HAL_GetTick();
}

void pt_port_log(const char *s)
{
    /* 200 Hz × 40 字节 ≈ 8 kB/s，921600 波特完全够；
       如果嫌阻塞，改成 DMA + 环形缓冲 */
    HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 2);
}

void pt_port_write(const uint8_t *buf, uint16_t len)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 2);
}

/* ---------------------------------------------------------------- INA226 */
static int i2c_wr(uint8_t addr, uint8_t reg, const uint8_t *buf, uint16_t len)
{
    return (HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr << 1), reg,
                              I2C_MEMADD_SIZE_8BIT,
                              (uint8_t *)buf, len, 10) == HAL_OK) ? 0 : -1;
}

static int i2c_rd(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len)
{
    return (HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr << 1), reg,
                             I2C_MEMADD_SIZE_8BIT,
                             buf, len, 10) == HAL_OK) ? 0 : -1;
}

void pt_port_read_bus(uint16_t *mv, int32_t *ma)
{
    if (ina226_read(mv, ma) != 0) {
        if (mv) *mv = 0;
        if (ma) *ma = 0;
    }
}

/* ---------------------------------------------------------------- 定时器 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM6) {
        pt_tick_1ms();
    }
}

/* ---------------------------------------------------------------- 启动 */
void app_power_test_start(void)
{
    pt_config_t cfg = {
        .motor_id      = 1,
        .i_target      = 2000,     /* 实验一先 2000 counts，别贪 */
        .idle_ms       = 2000,
        .ramp_ms       = 60000,    /* 60 秒爬完，接近准静态 */
        .hold_ms       = 3000,
        .log_period_ms = 5,        /* 200 Hz */
        .rpm_limit     = 9000,
        .temp_limit    = 80,
    };

    can_start();
    ina226_init(i2c_wr, i2c_rd, INA226_ADDR, INA_R_MOHM);

    pt_init(&cfg);
    pt_start();

    HAL_TIM_Base_Start_IT(&htim6);   /* 1 kHz */
}

/* 在 main() 里所有 MX_xxx_Init() 之后调用 app_power_test_start(); */
