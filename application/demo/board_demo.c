/**
 * @file board_demo.c
 * @brief 达妙 MC02 开发板示例应用(移植后的框架自检)
 */

#include "board_demo.h"
#include "robot_def.h"

#include <math.h>

#include "bsp_board.h"
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "bsp_adc.h"
#include "bsp_ws2812.h"

#include "ins_task.h"
#include "buzzer.h"
#include "daemon.h"
#include "vofa.h"
#if DEMO_DM_MOTOR_ENABLE
#include "dm_motor.h"
static DMMotorInstance *demo_dm_motor;
#endif

#if DEMO_DJI_MOTOR_ENABLE
#include "dji_motor.h"
static DJIMotorInstance *demo_dji_motor;
#endif

#if DEMO_REMOTE_ENABLE
#include "remote_control.h"
static RC_ctrl_t *demo_rc; // DR16/DT7 遥控器数据(接板载 SBUS 口)
#endif

static attitude_t *demo_imu;            // 姿态数据指针(指向 INS 任务内部结构)
#if DEMO_BUZZER_ENABLE
static float demo_last_key;             // 按键消抖时间戳
static uint8_t demo_first_melody = 1;   // 开机是否还要播一次示例旋律
#endif
#if DEMO_DM_MOTOR_ENABLE || DEMO_DJI_MOTOR_ENABLE
static float demo_sweep_phase; // 电机演示用的正弦相位
#endif

/* 旋律表放在 demo_songs.h 里(app 层拥有"歌", module 层只提供播放器) */
#if DEMO_BUZZER_ENABLE
#include "demo_songs.h"

/* 开机音乐: 三音上行, 从"全向轮"工程的 dji_starting_music 移植 */
#define DEMO_STARTUP_MUSIC demo_startup_music
#define DEMO_STARTUP_MUSIC_LEN ((uint16_t)ARRAY_LEN(demo_startup_music))

/* 按键演示旋律: 小星星(公版) */
#define DEMO_KEY_MELODY demo_melody
#define DEMO_KEY_MELODY_LEN ((uint16_t)ARRAY_LEN(demo_melody))

/* 按键循环播放的曲目表: 每按一次换一首, 方便试听不同音区/时值 */
typedef struct
{
    const BuzzerNote_t *notes;
    uint16_t count;
    const char *name;
} DemoSong_t;

static const DemoSong_t demo_songs[] = {
    /* 排第一: 按一下用户按键就直接播这首 30 秒的旋律 */
    { melody_yuai_sf, (uint16_t)ARRAY_LEN(melody_yuai_sf), "yuai-30s" },
    { demo_startup_music, (uint16_t)ARRAY_LEN(demo_startup_music), "startup" },
    { demo_melody, (uint16_t)ARRAY_LEN(demo_melody), "twinkle" },
    { demo_melody_ode, (uint16_t)ARRAY_LEN(demo_melody_ode), "ode-to-joy" },
};
static uint8_t demo_song_idx = 0;
#endif

/* 示例 app 的周期任务时间戳 */
static float demo_last_led;
static float demo_last_log;

void BoardDemoInit(void)
{
    /* ---------------- 1. 蜂鸣器 ---------------- */
    /* 旋律播放器直接用 BuzzerInit() 创建的那个 PWM 实例,
     * 所以这里不需要再注册 alarm 实例(避免和 BuzzerTask 抢同一个通道) */

#if DEMO_LED_ENABLE
    /* 上电指示(白): 看到白灯说明 bsp 已经跑通, 如果之后一直白着 = 卡在下面某一步 */
    BSPWS2812SetRGB(60, 60, 60);
#endif
    LOGINFO("[demo] stage 1/4 start (LED=white)");

    /* ---------------- 2. 姿态解算 ---------------- */
    /* INS_Init() 会初始化 BMI088(SPI2) 并等待其就绪, 然后做静置标定(最长 12 秒).
     * 这 12 秒内不会有新日志, 板子必须保持静止, 否则标定会被打断重来. */
#if DEMO_LED_ENABLE
    BSPWS2812SetRGB(0, 0, 60); /* 蓝: 正在初始化/标定 IMU */
#endif
    LOGINFO("[demo] stage 2/4 INS init (LED=blue, keep still up to 12s)");
    demo_imu = INS_Init();
    LOGINFO("[demo] stage 2/4 INS done");

#if DEMO_DM_MOTOR_ENABLE
    /* ---------------- 3. 达妙电机(可选) ---------------- */
    /* MIT 协议: 控制帧与反馈帧都用 ID=1; 这里用框架的"速度环+电流环"算出目标力矩,
     * 通过 t_ff 下发(kp=0 即纯力矩模式). */
    DMMotor_Init_Config_s dm_config = {
        .can_init_config = {
            .can_handle = BOARD_CAN1,
            .tx_id = 1, /* 会被 DMMotorInit 覆盖为 motor_id */
            .rx_id = 1,
            .nominal_bitrate = 1000000,
        },
        .motor_id = 1,
        .kp = 0.0f, /* 0: 力矩模式(由框架 PID 输出), >0: 电机内部位置环 */
        .kd = 0.0f,
        .controller_setting_init_config = {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 2.0f,
                .Ki = 0.5f,
                .Kd = 0.0f,
                .MaxOut = 3.0f,
                .IntegralLimit = 2.0f,
                .Improve = PID_Integral_Limit,
            },
            .current_PID = {
                .Kp = 1.0f,
                .Ki = 0.1f,
                .Kd = 0.0f,
                .MaxOut = 3.0f,
                .IntegralLimit = 1.0f,
                .Improve = PID_Integral_Limit,
            },
        },
    };
    demo_dm_motor = DMMotorInit(&dm_config);
    LOGINFO("[demo] dm motor id=1 registered on CAN1");
#endif

#if DEMO_DJI_MOTOR_ENABLE
    /* ---------------- 4. DJI 电机(可选) ---------------- */
    Motor_Init_Config_s dji_config = {
        .can_init_config = {
            .can_handle = BOARD_CAN2,
            .tx_id = 1,
            .rx_id = 0x201,
            .nominal_bitrate = 1000000,
        },
        .controller_setting_init_config = {
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP | CURRENT_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_NORMAL,
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .feedforward_flag = FEEDFORWARD_NONE,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 30.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = 16000.0f,
                .IntegralLimit = 8000.0f,
                .Improve = PID_Integral_Limit,
            },
            .current_PID = {
                .Kp = 1.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .MaxOut = 16384.0f,
                .Improve = PID_IMPROVE_NONE,
            },
        },
        .motor_type = M3508,
    };
    demo_dji_motor = DJIMotorInit(&dji_config);
    LOGINFO("[demo] dji M3508 id=1 registered on CAN2");
#endif

#if DEMO_LED_ENABLE
    BSPWS2812SetRGB(0, 60, 0); // 绿色: 初始化完成
#endif
    LOGINFO("[demo] stage 4/4 board demo app init done (LED=green)");

#if DEMO_REMOTE_ENABLE
    /* ---------------- 6. DR16 接收机(可选) ----------------
     * 接线: DR16 的 S.BUS 口 -> 板载 SBUS 口(3pin: S / 5V / GND)
     * 板上 SBUS 输入自带反相器, 接到 UART5_RX(PD2); UART5 已配置为 DBUS: 100000-8E1
     */
    demo_rc = RemoteControlInit(BOARD_UART5);
    LOGINFO("[demo] DR16/DBUS remote on UART5 (SBUS port)");

#endif

    LOGINFO("[demo] board demo app init done");
}

void BoardDemoTask(void)
{

    float now = DWT_GetTimeline_s();

    /* ---------------- 1. 彩灯心跳(1Hz 闪烁) ---------------- */
#if DEMO_LED_ENABLE
    if (now - demo_last_led >= 0.5f)
    {
        static uint8_t led_on = 0;
        demo_last_led = now;
        led_on = !led_on;
        BSPWS2812SetRGB(0, led_on ? 60 : 0, 0); // 绿色心跳
    }
#endif

    /* ---------------- 2. 蜂鸣器: 开机旋律 + 按键重播 ----------------
     * 无源蜂鸣器只能单音, 这里播放的是"旋律"(简谱), 不是 beatbox 鼓点.
     * 要换成别的曲子, 改上面的 demo_melody 表即可(换算规则见注释).
     */
#if DEMO_BUZZER_ENABLE
    /* BuzzerInit() 在 DAEMON 任务里执行, 等它准备好再播开机旋律 */
    if (demo_first_melody && BuzzerIsReady())
    {
        demo_first_melody = 0;
        /* 开机音乐(三音上行), 播放器非阻塞, 不会拖慢启动 */
        BuzzerPlayMelody(DEMO_STARTUP_MUSIC, DEMO_STARTUP_MUSIC_LEN, 0);
        LOGINFO("[demo] playing startup music");
    }

    /* 按下用户按键: 播放 / 停止 */
    if (BoardKeyPressed() && (now - demo_last_key > 0.4f))
    {
        demo_last_key = now;
        if (BuzzerIsPlaying())
        {
            BuzzerStopMelody();
            LOGINFO("[demo] key pressed: melody stopped");
        }
        else
        {
            /* 每按一次换下一首(播放器非阻塞, 直接切歌即可) */
            const DemoSong_t *song = &demo_songs[demo_song_idx];
            BuzzerPlayMelody(song->notes, song->count, 0);
            LOGINFO("[demo] key pressed: play '%s' (%d notes)", song->name, (int)song->count);
            demo_song_idx = (uint8_t)((demo_song_idx + 1U) % ARRAY_LEN(demo_songs));
        }
    }
#endif

    /* ---------------- 3. 电机闭环演示(可选) ---------------- */
#if DEMO_DM_MOTOR_ENABLE
    if (demo_dm_motor != NULL)
    {
        /* 目标速度按 0.2Hz 正弦变化: ±2 rad/s */
        demo_sweep_phase += 0.01f * 2.0f * 3.1415926f * 0.2f;
        DMMotorSetRef(demo_dm_motor, 2.0f * sinf(demo_sweep_phase));
    }
#endif

#if DEMO_DJI_MOTOR_ENABLE
    if (demo_dji_motor != NULL)
    {
        /* 目标转速按 0.2Hz 正弦变化: ±200 rpm */
        demo_sweep_phase += 0.01f * 2.0f * 3.1415926f * 0.2f;
        DJIMotorSetRef(demo_dji_motor, 200.0f * sinf(demo_sweep_phase));
    }
#endif

    /* ---------------- 4. VOFA+ 波形输出(100Hz) ---------------- */
#if DEMO_VOFA_ENABLE
    {
        float send[6] = {
            demo_imu->Yaw,
            demo_imu->Pitch,
            demo_imu->Roll,
            demo_imu->Gyro[2],
            ADCGetVbatVoltage(),
            0.0f,
        };
        vofa_justfloat_output(send, 6, BOARD_USART10);
    }
#endif

    /* ---------------- 5. 1Hz 日志 ---------------- */
    if (now - demo_last_log >= 1.0f)
    {
        demo_last_log = now;
        LOGINFO("[demo] yaw=%d pitch=%d roll=%d | vbat=%dmV | temp=%dC",
                (int)(demo_imu->Yaw * 100), (int)(demo_imu->Pitch * 100), (int)(demo_imu->Roll * 100),
                (int)(ADCGetVbatVoltage() * 1000), (int)ADCGetChipTemperature());
    }

#if DEMO_REMOTE_ENABLE
    /* ---------------- 6. 遥控器数据(5Hz) ---------------- */
    {
        static float demo_last_rc;
        if (demo_rc != NULL && (now - demo_last_rc >= 0.2f))
        {
            demo_last_rc = now;
            if (RemoteControlIsOnline())
            {
                LOGINFO("[demo] rc: r=%d/%d l=%d/%d dial=%d sw=%d/%d",
                        demo_rc->rc.rocker_r_, demo_rc->rc.rocker_r1,
                        demo_rc->rc.rocker_l_, demo_rc->rc.rocker_l1,
                        demo_rc->rc.dial,
                        demo_rc->rc.switch_left, demo_rc->rc.switch_right);
            }
            else
            {
                LOGWARNING("[demo] rc offline, check SBUS port / receiver power");
            }
        }
    }
#endif
}
