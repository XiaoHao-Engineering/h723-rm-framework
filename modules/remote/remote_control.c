#include "remote_control.h"
#include "string.h"
#include "bsp_usart.h"
#include "bsp_dwt.h"
#include "memory.h"
#include "stdlib.h"
#include "daemon.h"
#include "bsp_log.h"

#ifndef RC_UART_RX_INVERT
#define RC_UART_RX_INVERT 0 /* 1: UART5 反相接收(SBUS 是反相信号); 若接的是 DBUS 则设 0 */
#endif
#ifndef RC_DEBUG_RAW
#define RC_DEBUG_RAW 0      /* 1: 每秒打印一帧原始字节, 用于确认协议/极性 */
#endif
#define RC_SBUS_FRAME 25u
#define RC_SBUS_CH_OFFSET 992
#define REMOTE_CONTROL_FRAME_SIZE 25u // 遥控器接收的buffer大小

// 遥控器数据
static RC_ctrl_t rc_ctrl[2];     //[0]:当前数据TEMP,[1]:上一次的数据LAST.用于按键持续按下和切换的判断
static uint8_t rc_init_flag = 0; // 遥控器初始化标志位

// 遥控器拥有的串口实例,因为遥控器是单例,所以这里只有一个,就不封装了
static USARTInstance *rc_usart_instance;
static DaemonInstance *rc_daemon_instance;

/* 调试信息: 中断里只记录, 由任务上下文打印(中断里不能 LOG, 会爆栈) */
volatile uint32_t rc_dbg_cb = 0;      /* 回调次数 */
volatile uint16_t rc_dbg_len = 0;     /* 最近一次长度 */
volatile uint8_t  rc_dbg_head[8];     /* 最近一次前 8 字节 */

/**
 * @brief 矫正遥控器摇杆的值,超过660或者小于-660的值都认为是无效值,置0
 *
 */
static void RectifyRCjoystick()
{
    for (uint8_t i = 0; i < 5; ++i)
        if (abs(*(&rc_ctrl[TEMP].rc.rocker_l_ + i)) > 660)
            *(&rc_ctrl[TEMP].rc.rocker_l_ + i) = 0;
}

/**
 * @brief 遥控器数据解析
 *
 * @param sbus_buf 接收buffer
 */
/**
 * @brief 判断一段 18 字节数据是否像一帧合法 DBUS
 *
 * DBUS 没有帧头字节, 只能靠内容校验:
 *   - 两个开关只能是 1/2/3(两位编码, 不会是 0)
 *   - 四个摇杆通道应在有效范围内
 * 串口空闲中断给的块可能比 18 字节多(或多一个字节), 用这个函数滑窗对齐。
 */
static uint8_t dbus_frame_valid(const uint8_t *b)
{
    int16_t ch[4];
    uint8_t s1, s2;

    ch[0] = (int16_t)((b[0] | (b[1] << 8)) & 0x07FF);
    ch[1] = (int16_t)(((b[1] >> 3) | (b[2] << 5)) & 0x07FF);
    ch[2] = (int16_t)(((b[2] >> 6) | (b[3] << 2) | (b[4] << 10)) & 0x07FF);
    ch[3] = (int16_t)(((b[4] >> 1) | (b[5] << 7)) & 0x07FF);

    s1 = (uint8_t)(((b[5] >> 4) & 0x0C) >> 2);
    s2 = (uint8_t)((b[5] >> 4) & 0x03);

    if (s1 < 1u || s1 > 3u || s2 < 1u || s2 > 3u)
        return 0;

    for (uint8_t i = 0; i < 4; i++)
        if (ch[i] < 100 || ch[i] > 1900)
            return 0;

    return 1;
}

static void sbus_to_rc(const uint8_t *sbus_buf)
{
    // 摇杆,直接解算时减去偏置
    rc_ctrl[TEMP].rc.rocker_r_ = ((sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff) - RC_CH_VALUE_OFFSET;                              //!< Channel 0
    rc_ctrl[TEMP].rc.rocker_r1 = (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 1
    rc_ctrl[TEMP].rc.rocker_l_ = (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10)) & 0x07ff) - RC_CH_VALUE_OFFSET; //!< Channel 2
    rc_ctrl[TEMP].rc.rocker_l1 = (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff) - RC_CH_VALUE_OFFSET;                       //!< Channel 3
    rc_ctrl[TEMP].rc.dial = ((sbus_buf[16] | (sbus_buf[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET;                                 // 左侧拨轮
    RectifyRCjoystick();
    // 开关,0左1右
    rc_ctrl[TEMP].rc.switch_right = ((sbus_buf[5] >> 4) & 0x0003);     //!< Switch right
    rc_ctrl[TEMP].rc.switch_left = ((sbus_buf[5] >> 4) & 0x000C) >> 2; //!< Switch left

    // 鼠标解析
    rc_ctrl[TEMP].mouse.x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
    rc_ctrl[TEMP].mouse.y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
    rc_ctrl[TEMP].mouse.press_l = sbus_buf[12];                 //!< Mouse Left Is Press ?
    rc_ctrl[TEMP].mouse.press_r = sbus_buf[13];                 //!< Mouse Right Is Press ?

    //  位域的按键值解算,直接memcpy即可,注意小端低字节在前,即lsb在第一位,msb在最后
    *(uint16_t *)&rc_ctrl[TEMP].key[KEY_PRESS] = (uint16_t)(sbus_buf[14] | (sbus_buf[15] << 8));
    if (rc_ctrl[TEMP].key[KEY_PRESS].ctrl) // ctrl键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    if (rc_ctrl[TEMP].key[KEY_PRESS].shift) // shift键按下
        rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT] = rc_ctrl[TEMP].key[KEY_PRESS];
    else
        memset(&rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));

    uint16_t key_now = rc_ctrl[TEMP].key[KEY_PRESS].keys,                   // 当前按键是否按下
        key_last = rc_ctrl[LAST].key[KEY_PRESS].keys,                       // 上一次按键是否按下
        key_with_ctrl = rc_ctrl[TEMP].key[KEY_PRESS_WITH_CTRL].keys,        // 当前ctrl组合键是否按下
        key_with_shift = rc_ctrl[TEMP].key[KEY_PRESS_WITH_SHIFT].keys,      //  当前shift组合键是否按下
        key_last_with_ctrl = rc_ctrl[LAST].key[KEY_PRESS_WITH_CTRL].keys,   // 上一次ctrl组合键是否按下
        key_last_with_shift = rc_ctrl[LAST].key[KEY_PRESS_WITH_SHIFT].keys; // 上一次shift组合键是否按下

    for (uint16_t i = 0, j = 0x1; i < 16; j <<= 1, i++)
    {
        if (i == 4 || i == 5) // 4,5位为ctrl和shift,直接跳过
            continue;
        // 如果当前按键按下,上一次按键没有按下,且ctrl和shift组合键没有按下,则按键按下计数加1(检测到上升沿)
        if ((key_now & j) && !(key_last & j) && !(key_with_ctrl & j) && !(key_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS][i]++;
        // 当前ctrl组合键按下,上一次ctrl组合键没有按下,则ctrl组合键按下计数加1(检测到上升沿)
        if ((key_with_ctrl & j) && !(key_last_with_ctrl & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_CTRL][i]++;
        // 当前shift组合键按下,上一次shift组合键没有按下,则shift组合键按下计数加1(检测到上升沿)
        if ((key_with_shift & j) && !(key_last_with_shift & j))
            rc_ctrl[TEMP].key_count[KEY_PRESS_WITH_SHIFT][i]++;
    }

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t)); // 保存上一次的数据,用于按键持续按下和切换的判断
}

/**
 * @brief 对sbus_to_rc的简单封装,用于注册到bsp_usart的回调函数中
 *
 */
/* ---------------- SBUS(25 字节, 0x0F 帧头) ---------------- */
#if RC_DEBUG_RAW
static uint8_t cnt = 0; /* 日志节流, 仅调试用 */
#endif
static void sbus_to_rc_sbus(const uint8_t *b)
{
    int16_t ch[16];

    ch[0]  = ((b[1]       | b[2]  << 8) & 0x07FF);
    ch[1]  = ((b[2] >> 3  | b[3]  << 5) & 0x07FF);
    ch[2]  = ((b[3] >> 6  | b[4]  << 2 | b[5] << 10) & 0x07FF);
    ch[3]  = ((b[5] >> 1  | b[6]  << 7) & 0x07FF);
    ch[4]  = ((b[6] >> 4  | b[7]  << 4) & 0x07FF);
    ch[5]  = ((b[7] >> 7  | b[8]  << 1 | b[9] << 9) & 0x07FF);
    ch[6]  = ((b[9] >> 2  | b[10] << 6) & 0x07FF);
    ch[7]  = ((b[10] >> 5 | b[11] << 3) & 0x07FF);

    rc_ctrl[TEMP].rc.rocker_r_  = ch[0] - RC_SBUS_CH_OFFSET;
    rc_ctrl[TEMP].rc.rocker_r1  = ch[1] - RC_SBUS_CH_OFFSET;
    rc_ctrl[TEMP].rc.rocker_l_  = ch[2] - RC_SBUS_CH_OFFSET;
    rc_ctrl[TEMP].rc.rocker_l1  = ch[3] - RC_SBUS_CH_OFFSET;
    rc_ctrl[TEMP].rc.dial       = ch[6] - RC_SBUS_CH_OFFSET; /* 先按 ch7 当拨轮, 看日志再定 */
    RectifyRCjoystick();

    /* 开关: SBUS 三档约 172 / 992 / 1811, 先按"高=上, 中=中, 低=下"映射 */
    rc_ctrl[TEMP].rc.switch_left  = (ch[4] > 1400) ? RC_SW_UP : (ch[4] < 600 ? RC_SW_DOWN : RC_SW_MID);
    rc_ctrl[TEMP].rc.switch_right = (ch[5] > 1400) ? RC_SW_UP : (ch[5] < 600 ? RC_SW_DOWN : RC_SW_MID);

    memset(&rc_ctrl[TEMP].mouse, 0, sizeof(rc_ctrl[TEMP].mouse));
    memset(rc_ctrl[TEMP].key, 0, sizeof(rc_ctrl[TEMP].key));

    memcpy(&rc_ctrl[LAST], &rc_ctrl[TEMP], sizeof(RC_ctrl_t));

#if RC_DEBUG_RAW
    if (++cnt >= 5) /* 约 5 帧打印一次 */
    {
        cnt = 0;
        LOGINFO("[rc] SBUS ch=%d,%d,%d,%d,%d,%d,%d,%d hdr=%02X tail=%02X",
                ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7], b[0], b[24]);
    }
#endif
}

static void RemoteControlRxCallback()
{
    const uint8_t *buf = rc_usart_instance->recv_buff;
    const uint16_t len = rc_usart_instance->recv_len;

    /* 只记录, 不打印(中断上下文里 LOG 会爆栈) */
    rc_dbg_cb++;
    rc_dbg_len = len;
    for (uint8_t k = 0; k < 8; k++)
        rc_dbg_head[k] = buf[k];

    if (len >= RC_SBUS_FRAME && buf[0] == 0x0Fu)
    {
        sbus_to_rc_sbus(buf);
        DaemonReload(rc_daemon_instance);
        return;
    }

    /* DBUS: 逐个偏移尝试, 找到内容合法的 18 字节窗口(解决帧边界偏移) */
    for (uint16_t off = 0; (uint16_t)(off + 18u) <= len; off++)
    {
        if (dbus_frame_valid(&buf[off]))
        {
            sbus_to_rc(&buf[off]);
            DaemonReload(rc_daemon_instance);
            return;
        }
    }
    /* 没找到合法帧: 不喂狗, 让守护进程判离线 */
}

/**
 * @brief 遥控器离线的回调函数,注册到守护进程中,串口掉线时调用
 *
 */
static void RCLostCallback(void *id)
{
    /* 接收机没接的时候这个回调每次超时都会进来(约 10Hz),
     * 不限速的话会把 RTT 日志缓冲冲爆, 把启动日志和真正的故障信息挤掉 */
    static float last_log = 0.0f;
    float now = DWT_GetTimeline_ms();

    memset(rc_ctrl, 0, sizeof(rc_ctrl)); // 清空遥控器数据
    USARTServiceInit(rc_usart_instance); // 尝试重新启动接收

    if ((now - last_log) > 2000.0f)
    {
        last_log = now;
        LOGWARNING("[rc] remote control lost");
    }
}

RC_ctrl_t *RemoteControlInit(UART_HandleTypeDef *rc_usart_handle)
{
    USART_Init_Config_s conf;
    conf.module_callback = RemoteControlRxCallback;
    conf.usart_handle = rc_usart_handle;
    conf.recv_buff_size = REMOTE_CONTROL_FRAME_SIZE;
#if RC_UART_RX_INVERT
    /* SBUS 是反相信号: 打开 UART 的 RX 反相(在启动 DMA 接收之前改) */
    rc_usart_handle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT;
    rc_usart_handle->AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
    (void)HAL_UART_Init(rc_usart_handle);
#endif

    rc_usart_instance = USARTRegister(&conf);

    // 进行守护进程的注册,用于定时检查遥控器是否正常工作
    Daemon_Init_Config_s daemon_conf = {
        .reload_count = 10, // 100ms未收到数据视为离线,遥控器的接收频率实际上是1000/14Hz(大约70Hz)
        .callback = RCLostCallback,
        .owner_id = NULL, // 只有1个遥控器,不需要owner_id
    };
    rc_daemon_instance = DaemonRegister(&daemon_conf);

    rc_init_flag = 1;
    return rc_ctrl;
}

uint8_t RemoteControlIsOnline()
{
    if (rc_init_flag)
        return DaemonIsOnline(rc_daemon_instance);
    return 0;
}