#ifndef BUZZER_H
#define BUZZER_H
#include "bsp_pwm.h"
#define BUZZER_DEVICE_CNT 5

#define  DoFreq  523
#define  ReFreq  587
#define  MiFreq  659
#define  FaFreq  698
#define  SoFreq  784
#define  LaFreq  880
#define  SiFreq  988

typedef enum
{
    OCTAVE_1 = 0,
    OCTAVE_2,
    OCTAVE_3,
    OCTAVE_4,
    OCTAVE_5,
    OCTAVE_6,
    OCTAVE_7,
    OCTAVE_8,
}octave_e;

typedef enum
{
    ALARM_LEVEL_HIGH = 0,
    ALARM_LEVEL_ABOVE_MEDIUM,
    ALARM_LEVEL_MEDIUM,
    ALARM_LEVEL_BELOW_MEDIUM,
    ALARM_LEVEL_LOW,
}AlarmLevel_e;

typedef enum
{
    ALARM_OFF = 0,
    ALARM_ON,
}AlarmState_e;
typedef struct
{
    AlarmLevel_e alarm_level;
    octave_e octave;
    float loudness;
}Buzzer_config_s;

typedef struct
{
    float loudness;
    octave_e octave;
    AlarmLevel_e alarm_level;
    AlarmState_e alarm_state;
}BuzzzerInstance;


void BuzzerInit();
void BuzzerTask();
BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config);
void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state);

/* ============================ 简单旋律播放 ============================ */
/**
 * 无源蜂鸣器只能发出"一个音高的方波", 所以:
 *   - 能还原单音旋律(简谱), 还原不了和声/鼓点/气声;
 *   - 频率决定音高, 占空比决定响度(50% 最响).
 *
 * 用法:
 *   1. 把简谱写成 BuzzerNote_t 数组(频率 Hz + 时长 ms, 频率 0 = 休止);
 *   2. 调用 BuzzerPlayMelody(notes, count, loop), 立刻返回, 不阻塞;
 *   3. 在 10ms 周期任务里调用 BuzzerMusicTask() 推进播放.
 */

/* 频率整体上移倍数: MC02 的蜂鸣器谐振在 ~4kHz, 音符太低会明显变轻,
 * 乘 4 可以让 C4(262Hz) 落到 1046Hz, 听感更清楚; 想更响可以改成 8 */
#define BUZZER_MUSIC_TRANSPOSE 4

typedef struct
{
    uint16_t freq_hz;    /* 音符频率(Hz), 0 表示休止符 */
    uint16_t duration_ms; /* 持续时间(ms) */
} BuzzerNote_t;

/**
 * @brief 播放一段旋律(非阻塞)
 * @param notes 音符表
 * @param count 音符个数
 * @param loop  1: 循环播放, 0: 播完自动停
 */
void BuzzerPlayMelody(const BuzzerNote_t *notes, uint16_t count, uint8_t loop);

/**
 * @brief 停止旋律播放
 */
void BuzzerStopMelody(void);

/**
 * @brief 设置旋律播放速度
 *
 * @param scale 1.0 = 谱面原速; 2.0 = 快一倍; 0.5 = 慢一倍 (内部限制在 0.25~4.0)
 */
void BuzzerSetMelodyTempo(float scale);

/**
 * @brief 旋律推进任务, 请在 10ms 周期的任务里调用(与 BuzzerTask 同一处即可)
 */
void BuzzerMusicTask(void);

/**
 * @brief 是否正在播放旋律
 */
uint8_t BuzzerIsPlaying(void);

/**
 * @brief 蜂鸣器 PWM 是否已经初始化完成(BuzzerInit 之后)
 */
uint8_t BuzzerIsReady(void);

#endif // !BUZZER_H
