/**
 * @file bsp_can.c
 * @brief FDCAN 板级支持包(H7 版)
 *
 * 与 F4 版本(bxCAN)的主要区别:
 *  1. 外设从 bxCAN 换成 FDCAN, 因此过滤器/邮箱的概念变成 "消息RAM + RX FIFO + TX FIFO";
 *     F4 中"按 rx_id 添加过滤器"的做法在 H7 上会给消息RAM 分配带来额外复杂度,
 *     这里改为: 全局滤波器接收所有标准帧到 RX FIFO0, 在中断里按 (句柄, rx_id) 分发,
 *     对 module 层暴露的语义完全不变.
 *  2. 支持 CANFD(最长64字节数据 / BRS), 达妙电机的 FD 协议可以直接使用.
 *  3. 位时序在注册时按实际 FDCAN 时钟自动计算, 不依赖 CubeMX 里填的数值.
 */

#ifndef CAN_LOOPBACK_SELFTEST
#define CAN_LOOPBACK_SELFTEST 1 /* 1: 上电做一次 CAN3 内部回环自检 */
#endif

#ifndef CAN_DEBUG_UNMATCHED
#define CAN_DEBUG_UNMATCHED 0 /* 1: 打印总线上未被任何实例匹配的帧 ID(诊断用) */
#endif

#include "bsp_can.h"
#include "bsp_board.h"
#include "bsp_dwt.h"
#include "bsp_log.h"

#include <stdlib.h>
#include <string.h>

/* CAN 实例表, 接收中断里遍历该表完成分发 */
static CANInstance *can_instance[CAN_MX_REGISTER_CNT] = {NULL};
static uint8_t idx; // 全局CAN实例索引,每次有新的模块注册会自增

/* 每条总线的状态 */
typedef struct
{
    FDCAN_HandleTypeDef *handle;
    uint8_t inited;
    uint8_t is_fd;
} CANBusState_t;

static CANBusState_t can_bus[DEVICE_CAN_CNT];

/* Bus-Off 事件计数/待处理标志: 中断里只记事件, 恢复动作放到任务上下文(见 CANTransmit) */
static volatile uint8_t busoff_pending[DEVICE_CAN_CNT] = {0};
static volatile uint32_t busoff_count[DEVICE_CAN_CNT] = {0};
static float busoff_last_restart[DEVICE_CAN_CNT] = {0};
static float busoff_last_log = 0;

/* ----------------------------- 内部工具函数 ----------------------------- */

/**
 * @brief 由句柄取得总线下标(0/1/2), 未知句柄返回 0xFF
 */
static uint8_t CANBusIndex(FDCAN_HandleTypeDef *handle)
{
    if (handle == NULL)
        return 0xFF;
    if (handle->Instance == FDCAN1)
        return 0;
    if (handle->Instance == FDCAN2)
        return 1;
    if (handle->Instance == FDCAN3)
        return 2;
    return 0xFF;
}

/**
 * @brief FDCAN DLC 编码 → 实际字节数
 */
static uint8_t CANDlcToLen(uint32_t dlc)
{
    static const uint8_t map[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return (dlc < 16) ? map[dlc] : 0;
}

/**
 * @brief 实际字节数 → FDCAN DLC 编码
 */
static uint32_t CANLenToDlc(uint8_t len)
{
    if (len <= 8)
        return len;
    if (len <= 12)
        return 9;
    if (len <= 16)
        return 10;
    if (len <= 20)
        return 11;
    if (len <= 24)
        return 12;
    if (len <= 32)
        return 13;
    if (len <= 48)
        return 14;
    return 15;
}

/**
 * @brief 根据 FDCAN 内核时钟自动计算一组位时序参数(采样点约 80%)
 *
 * @param fclk         FDCAN 内核时钟(Hz)
 * @param bitrate      目标波特率
 * @param prescaler_max 预分频上限(仲裁段 512, 数据段 32)
 * @param psc / seg1 / seg2 输出参数
 * @return uint8_t 1 成功, 0 失败
 */
static uint8_t CANCalcTiming(uint32_t fclk, uint32_t bitrate, uint32_t prescaler_max,
                             uint32_t *psc, uint32_t *seg1, uint32_t *seg2)
{
    uint32_t best_err = 0xFFFFFFFFu;
    uint32_t best_psc = 0, best_seg1 = 0, best_seg2 = 0;

    if (fclk == 0 || bitrate == 0)
        return 0;

    for (uint32_t p = 1; p <= prescaler_max; ++p)
    {
        uint32_t tq_total = fclk / (p * bitrate);
        if (tq_total < 8 || tq_total > 385)
            continue;

        uint32_t actual = fclk / (p * tq_total);
        uint32_t err = (actual > bitrate) ? (actual - bitrate) : (bitrate - actual);

        if (err < best_err)
        {
            /* 采样点设在 80%: (1 + seg1) / tq_total ≈ 0.8 */
            uint32_t s1 = (tq_total * 4) / 5;
            if (s1 < 2)
                s1 = 2;
            if (s1 > tq_total - 2)
                s1 = tq_total - 2;
            uint32_t s2 = tq_total - 1 - s1;
            if (s2 < 2)
                s2 = 2;

            best_err = err;
            best_psc = p;
            best_seg1 = s1;
            best_seg2 = s2;
        }
        if (best_err == 0)
            break;
    }

    if (best_psc == 0)
        return 0;

    *psc = best_psc;
    *seg1 = best_seg1;
    *seg2 = best_seg2;
    return 1;
}

/**
 * @brief 按需求重新配置一条 FDCAN 总线(位时序 + 消息RAM + 滤波器 + 中断)
 *
 * @note 每条总线预留 0x300(768) 个消息RAM 字, H723 共有 2560 字, 三条总线足够.
 */
static void CANBusHardwareInit(FDCAN_HandleTypeDef *handle, uint8_t bus_idx, uint8_t is_fd,
                               uint32_t nominal_bitrate, uint32_t data_bitrate)
{
    uint32_t fclk = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
    uint32_t psc = 1, seg1 = 2, seg2 = 2;
    uint32_t d_psc = 1, d_seg1 = 2, d_seg2 = 2;

    if (nominal_bitrate == 0)
        nominal_bitrate = 1000000; /* 默认 1Mbps, DJI 电调与达妙电机都支持 */
    if (data_bitrate == 0)
        data_bitrate = 5000000; /* CANFD 数据段默认 5Mbps */

    if (!CANCalcTiming(fclk, nominal_bitrate, 512, &psc, &seg1, &seg2))
    {
        LOGERROR("[bsp_can] can not find a valid nominal timing, fclk=%d, bitrate=%d", (int)fclk, (int)nominal_bitrate);
        return;
    }

    if (is_fd)
    {
        if (!CANCalcTiming(fclk, data_bitrate, 32, &d_psc, &d_seg1, &d_seg2))
        {
            LOGWARNING("[bsp_can] can not reach %d bps for data phase, fall back to classic CAN", (int)data_bitrate);
            is_fd = 0;
            d_psc = 1;
            d_seg1 = 2;
            d_seg2 = 2;
        }
    }

    /* 校验实际可达波特率: 例如 96MHz 的 FDCAN 时钟下无法精确得到 5Mbps 的数据段 */
    {
        uint32_t actual_nominal = fclk / (psc * (1U + seg1 + seg2));
        uint32_t diff = (actual_nominal > nominal_bitrate) ? (actual_nominal - nominal_bitrate)
                                                           : (nominal_bitrate - actual_nominal);
        if (diff > nominal_bitrate / 100U)
            LOGWARNING("[bsp_can] nominal bitrate %d bps not reachable, closest is %d bps",
                       (int)nominal_bitrate, (int)actual_nominal);

        if (is_fd)
        {
            uint32_t actual_data = fclk / (d_psc * (1U + d_seg1 + d_seg2));
            uint32_t ddiff = (actual_data > data_bitrate) ? (actual_data - data_bitrate)
                                                          : (data_bitrate - actual_data);
            if (ddiff > data_bitrate / 100U)
                LOGWARNING("[bsp_can] data bitrate %d bps not reachable, closest is %d bps (fdcan clock=%d Hz)",
                           (int)data_bitrate, (int)actual_data, (int)fclk);
        }
    }

    /* 先反初始化: CubeMX 生成的位时序是按 ioc 里填的值算的, 这里统一按运行时计算的结果配置 */
    HAL_FDCAN_DeInit(handle);
    FDCAN_InitTypeDef *init = &handle->Init;
    init->FrameFormat = is_fd ? FDCAN_FRAME_FD_BRS : FDCAN_FRAME_CLASSIC;
    init->Mode = FDCAN_MODE_NORMAL;
    init->AutoRetransmission = DISABLE;
    init->TransmitPause = DISABLE;
    init->ProtocolException = DISABLE;

    init->NominalPrescaler = psc;
    init->NominalSyncJumpWidth = 1;
    init->NominalTimeSeg1 = seg1;
    init->NominalTimeSeg2 = seg2;

    init->DataPrescaler = d_psc;
    init->DataSyncJumpWidth = 1;
    init->DataTimeSeg1 = d_seg1;
    init->DataTimeSeg2 = d_seg2;

    /* 每条总线在消息RAM 中占 0x300 字 */
    init->MessageRAMOffset = bus_idx * 0x300;

    /* 不使用过滤器元素: 采用全局滤波器接收全部标准帧, 在中断里按 id 分发 */
    init->StdFiltersNbr = 0;
    init->ExtFiltersNbr = 0;

    init->RxFifo0ElmtsNbr = 16;
    init->RxFifo0ElmtSize = FDCAN_DATA_BYTES_64;
    init->RxFifo1ElmtsNbr = 0;
    init->RxFifo1ElmtSize = FDCAN_DATA_BYTES_64;
    init->RxBuffersNbr = 0;
    init->RxBufferSize = FDCAN_DATA_BYTES_64;

    init->TxEventsNbr = 0;
    init->TxBuffersNbr = 0;
    init->TxFifoQueueElmtsNbr = 16;
    init->TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    init->TxElmtSize = FDCAN_DATA_BYTES_64;

    if (HAL_FDCAN_Init(handle) != HAL_OK)
    {
        LOGERROR("[bsp_can] FDCAN%d init failed", bus_idx + 1);
        return;
    }

    /* 全局滤波: 匹配/不匹配的标准帧都收进 FIFO0, 远端帧一律丢弃 */
    if (HAL_FDCAN_ConfigGlobalFilter(handle, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
    {
        LOGERROR("[bsp_can] FDCAN%d global filter config failed", bus_idx + 1);
    }

    if (HAL_FDCAN_Start(handle) != HAL_OK)
    {
        LOGERROR("[bsp_can] FDCAN%d start failed", bus_idx + 1);
        return;
    }

    HAL_FDCAN_ActivateNotification(handle, FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF, 0);

    can_bus[bus_idx].inited = 1;
    can_bus[bus_idx].is_fd = is_fd;

    LOGINFO("[bsp_can] FDCAN%d init: %s %d/%d bps (fclk=%d, psc=%d, seg1=%d, seg2=%d)",
            bus_idx + 1, is_fd ? "FD" : "classic", (int)nominal_bitrate, (int)data_bitrate,
            (int)fclk, (int)psc, (int)seg1, (int)seg2);
}

/* ------------------------- 对外接口 ------------------------- */

CANInstance *CANRegister(CAN_Init_Config_s *config)
{
    uint8_t bus_idx = CANBusIndex(config->can_handle);
    if (bus_idx >= DEVICE_CAN_CNT)
    {
        while (1)
            LOGERROR("[bsp_can] CANRegister: illegal can handle, use BOARD_CANx macro");
    }

    if (idx >= CAN_MX_REGISTER_CNT) // 超过最大实例数
    {
        while (1)
            LOGERROR("[bsp_can] CAN instance exceeded MAX num, consider balance the load of CAN bus");
    }

    /* 同一条总线上的第一个实例负责初始化硬件 */
    if (!can_bus[bus_idx].inited)
    {
        CANBusHardwareInit(config->can_handle, bus_idx, config->is_fd,
                           config->nominal_bitrate, config->data_bitrate);
    }
    else if (can_bus[bus_idx].is_fd != (config->is_fd ? 1 : 0))
    {
        /* 同一总线上的设备必须使用相同的帧类型, 否则会出现收发异常 */
        LOGWARNING("[bsp_can] FDCAN%d already configured as %s, but this instance requests %s",
                   bus_idx + 1, can_bus[bus_idx].is_fd ? "FD" : "classic", config->is_fd ? "FD" : "classic");
    }

    /* 重复注册 | 同一个总线上 rx_id 冲突 */
    for (uint8_t i = 0; i < idx; ++i)
    {
        if (can_instance[i]->rx_id == config->rx_id && can_instance[i]->can_handle == config->can_handle)
        {
            while (1)
                LOGERROR("[bsp_can] CAN rx_id [0x%x] already registered on this bus", (int)config->rx_id);
        }
    }

    CANInstance *instance = (CANInstance *)malloc(sizeof(CANInstance));
    memset(instance, 0, sizeof(CANInstance));

    instance->can_handle = config->can_handle;
    instance->tx_id = config->tx_id;
    instance->rx_id = config->rx_id;
    instance->can_module_callback = config->can_module_callback;
    instance->id = config->id;
    instance->is_fd = config->is_fd ? 1 : 0;
    instance->brs = config->brs ? 1 : 0;
    instance->tx_len = 8;

    /* 发送报文头, 每次发送时只需更新 DataLength */
    instance->txconf.Identifier = config->tx_id;
    instance->txconf.IdType = FDCAN_STANDARD_ID;
    instance->txconf.TxFrameType = FDCAN_DATA_FRAME;
    instance->txconf.DataLength = CANLenToDlc(instance->tx_len);
    instance->txconf.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    instance->txconf.BitRateSwitch = instance->brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    instance->txconf.FDFormat = instance->is_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    instance->txconf.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    instance->txconf.MessageMarker = 0;

    can_instance[idx++] = instance;
    return instance;
}

void CANSetDLC(CANInstance *_instance, uint8_t length)
{
    uint8_t max_len = _instance->is_fd ? CAN_MAX_DATA_LEN : 8;
    if (length > max_len || length == 0) // 安全检查
        while (1)
            LOGERROR("[bsp_can] CAN DLC error! len=%d, max=%d, check your code or wild pointer",
                     (int)length, (int)max_len);

    _instance->tx_len = length;
}

void CANSetFrameType(CANInstance *_instance, uint8_t is_fd, uint8_t brs)
{
    _instance->is_fd = is_fd ? 1 : 0;
    _instance->brs = brs ? 1 : 0;
    _instance->txconf.FDFormat = _instance->is_fd ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
    _instance->txconf.BitRateSwitch = _instance->brs ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
}

uint8_t CANTransmit(CANInstance *_instance, float timeout)
{
    static uint32_t busy_count;
    static uint32_t tx_fail_count;
    static float tx_fail_last_log;
    float dwt_start = DWT_GetTimeline_ms();
    FDCAN_HandleTypeDef *handle = _instance->can_handle;
    uint8_t bus = CANBusIndex(handle);

    /* ---- Bus-Off 恢复(在任务上下文做, 不在中断里做) ----
     * 原来是在 HAL_FDCAN_ErrorStatusCallback 里直接 Stop/Start + 打印日志:
     *   1) LOGWARNING 是阻塞式串口发送, 在中断里打日志等于给中断风暴火上浇油;
     *   2) 总线真的坏了的时候, 在中断里反复 Stop/Start 会把一次 bus-off
     *      放大成"一直进中断"的死循环。
     * 现在中断里只置位, 这里每 100ms 最多尝试恢复一次。 */
    if (bus < DEVICE_CAN_CNT && busoff_pending[bus] &&
        (dwt_start - busoff_last_restart[bus]) > 100.0f)
    {
        busoff_pending[bus] = 0;
        busoff_last_restart[bus] = dwt_start;
        HAL_FDCAN_Stop(handle);
        HAL_FDCAN_Start(handle);

        /* 日志同样限速 1Hz: 总线没接好时 10Hz 的"bus-off"日志也会把串口刷满 */
        if ((dwt_start - busoff_last_log) > 1000.0f)
        {
            busoff_last_log = dwt_start;
            LOGWARNING("[bsp_can] FDCAN%d bus-off (cnt=%d), restarted. check CAN_H/L, 120ohm, ESC power",
                       bus + 1, (int)busoff_count[bus]);
        }
    }

    /* 等待发送 FIFO 有空位 */
    while (HAL_FDCAN_GetTxFifoFreeLevel(handle) == 0)
    {
        if (DWT_GetTimeline_ms() - dwt_start > timeout)
        {
            /* 限速 1Hz: 这条路径在总线挂掉时每个控制周期(1kHz)都会被走到,
             * 不限速的话一行 ~60 字节 × 1000 = 60kB/s, 115200 的串口直接堵死,
             * 连带把 1kHz 的电机任务拖慢 —— 表现就是"板子像卡死了"。 */
            tx_fail_count++;
            if ((DWT_GetTimeline_ms() - tx_fail_last_log) > 1000.0f)
            {
                tx_fail_last_log = DWT_GetTimeline_ms();
                LOGWARNING("[bsp_can] CAN TX fifo full (fail=%d), check wiring/termination/ESC power",
                           (int)tx_fail_count);
            }
            busy_count++;
            return 0;
        }
    }

    _instance->txconf.DataLength = CANLenToDlc(_instance->tx_len);

    if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &_instance->txconf, _instance->tx_buff) != HAL_OK)
    {
        /* 同样限速: 出错时这个分支每个控制周期都会进一次 */
        tx_fail_count++;
        if ((DWT_GetTimeline_ms() - tx_fail_last_log) > 1000.0f)
        {
            tx_fail_last_log = DWT_GetTimeline_ms();
            LOGWARNING("[bsp_can] CAN bus BUSY/ERROR (fail=%d), check wiring/termination",
                       (int)tx_fail_count);
        }
        busy_count++;
        return 0;
    }
    return 1; // 发送成功
}

/* ----------------------- 中断回调 ----------------------- */

/**
 * @brief 从指定 FIFO 中取出所有报文, 并按 (总线, rx_id) 分发给注册的实例
 */
#if CAN_ID_CENSUS
volatile uint16_t can_census_id[16];
volatile uint32_t can_census_cnt[16];
volatile uint8_t can_census_num = 0;

static void CAN_RecordId(uint32_t id)
{
    uint8_t i;

    for (i = 0; i < can_census_num; ++i)
    {
        if (can_census_id[i] == id)
        {
            can_census_cnt[i]++;
            return;
        }
    }
    if (can_census_num < 16)
    {
        can_census_id[can_census_num] = (uint16_t)id;
        can_census_cnt[can_census_num] = 1;
        can_census_num++;
    }
}
#endif

static void CANRxDispatch(FDCAN_HandleTypeDef *handle, uint32_t fifo)
{
    FDCAN_RxHeaderTypeDef rxconf;
    uint8_t rx_buff[CAN_MAX_DATA_LEN];

    while (HAL_FDCAN_GetRxFifoFillLevel(handle, fifo) > 0)
    {
        if (HAL_FDCAN_GetRxMessage(handle, fifo, &rxconf, rx_buff) != HAL_OK)
            break;

#if CAN_ID_CENSUS
        CAN_RecordId(rxconf.Identifier);
#endif

        if (rxconf.IdType != FDCAN_STANDARD_ID)
            continue; /* 目前只使用标准帧 */

        uint8_t len = CANDlcToLen(rxconf.DataLength);
        if (len > CAN_MAX_DATA_LEN)
            len = CAN_MAX_DATA_LEN;

        uint8_t matched = 0;

        for (uint8_t i = 0; i < idx; ++i)
        {
            if (handle == can_instance[i]->can_handle && rxconf.Identifier == can_instance[i]->rx_id)
            {
                can_instance[i]->rx_len = len;
                memcpy(can_instance[i]->rx_buff, rx_buff, len);
                if (can_instance[i]->can_module_callback != NULL)
                    can_instance[i]->can_module_callback(can_instance[i]);
                matched = 1;
                break;
            }
        }
        (void)matched; /* 嗅探关闭时避免未使用告警 */

#if CAN_DEBUG_UNMATCHED
        if (!matched)
        {
            static uint32_t seen_id[8];
            static uint32_t seen_hits[8];
            static uint8_t seen_cnt;

            uint8_t k, found = 0;

            for (k = 0; k < seen_cnt; ++k)
            {
                if (seen_id[k] == rxconf.Identifier)
                {
                    seen_hits[k]++;
                    found = 1;
                    break;
                }
            }
            if (!found && seen_cnt < 8)
            {
                seen_id[seen_cnt] = rxconf.Identifier;
                seen_hits[seen_cnt] = 1;
                LOGWARNING("[can] unmatched id 0x%x len %d (bus!)", (int)rxconf.Identifier, (int)len);
                seen_cnt++;
            }
        }
#endif
    }
}

/**
 * @brief RX FIFO0 有新报文(HAL 弱函数重载)
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    CANRxDispatch(hfdcan, FDCAN_RX_FIFO0);
}

/**
 * @brief RX FIFO1 有新报文(HAL 弱函数重载)
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    (void)RxFifo1ITs;
    CANRxDispatch(hfdcan, FDCAN_RX_FIFO1);
}

/**
 * @brief 总线错误回调: 出现 Bus-Off 时只记事件, 恢复动作在 CANTransmit 里做
 *
 * @note 千万不要在这里打日志/做阻塞操作: 这个回调本身就在中断里,
 *       一旦总线反复 bus-off, 一个 20ms 的阻塞串口发送就会把 CPU 吃光。
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0)
    {
        uint8_t bus = CANBusIndex(hfdcan);
        if (bus < DEVICE_CAN_CNT)
        {
            busoff_count[bus]++;
            busoff_pending[bus] = 1;
        }
    }
}

/**
 * @brief CAN 控制器内部回环自检(不外发, 不会驱动电机)
 *
 * 把控制器临时切到 Internal Loopback: 发一帧、自己收一帧, 收完切回原模式。
 * 用来区分"MCU 侧 CAN 控制器/时钟/引脚配置" 与 "外部收发器/接线" 的问题。
 *
 * @return 1=回环成功(MCU 侧正常)  0=失败(配置或时钟有问题)
 */
uint8_t BSPCAN_InternalLoopbackTest(FDCAN_HandleTypeDef *handle, uint32_t test_id)
{
    FDCAN_InitTypeDef saved;
    FDCAN_TxHeaderTypeDef tx;
    FDCAN_RxHeaderTypeDef rx;
    uint8_t data[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t ok = 0;

    if (handle == NULL)
        return 0;

    saved = handle->Init;

    HAL_FDCAN_Stop(handle);
    handle->Init.Mode = FDCAN_MODE_INTERNAL_LOOPBACK;
    if (HAL_FDCAN_Init(handle) != HAL_OK)
    {
        handle->Init = saved;
        (void)HAL_FDCAN_Init(handle);
        (void)HAL_FDCAN_Start(handle);
        return 0;
    }
    (void)HAL_FDCAN_Start(handle);

    tx.Identifier = test_id;
    tx.IdType = FDCAN_STANDARD_ID;
    tx.TxFrameType = FDCAN_DATA_FRAME;
    tx.DataLength = FDCAN_DLC_BYTES_8;
    tx.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx.BitRateSwitch = FDCAN_BRS_OFF;
    tx.FDFormat = FDCAN_CLASSIC_CAN;
    tx.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx.MessageMarker = 0;

    if (HAL_FDCAN_AddMessageToTxFifoQ(handle, &tx, data) == HAL_OK)
    {
        for (volatile uint32_t i = 0; i < 400000u; ++i)
        {
            if (HAL_FDCAN_GetRxFifoFillLevel(handle, FDCAN_RX_FIFO0) > 0)
            {
                if (HAL_FDCAN_GetRxMessage(handle, FDCAN_RX_FIFO0, &rx, data) == HAL_OK)
                    ok = (rx.Identifier == test_id) ? 1u : 0u;
                break;
            }
        }
    }

    HAL_FDCAN_Stop(handle);
    handle->Init = saved;
    (void)HAL_FDCAN_Init(handle);
    (void)HAL_FDCAN_Start(handle);
    return ok;
}
