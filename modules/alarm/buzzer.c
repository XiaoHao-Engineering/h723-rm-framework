#include "bsp_pwm.h"
#include "buzzer.h"
#include "bsp_dwt.h"
#include "bsp_board.h"
#include "bsp_log.h"
#include "string.h"

static PWMInstance *buzzer;
// static uint8_t idx;
static BuzzzerInstance *buzzer_list[BUZZER_DEVICE_CNT] = {0};

/**
 * @brief 蜂鸣器初始化
 * @note  MC02 的蜂鸣器接在 PB15 / TIM12_CH2, 由 bsp_board 初始化定时器
 */
void BuzzerInit()
{
    uint32_t channel = TIM_CHANNEL_2;
    TIM_HandleTypeDef *htim = BoardPWMGetHandle(BOARD_PWM_BUZZER, &channel);
    if (htim == NULL)
    {
        LOGERROR("[buzzer] can not get pwm handle");
        return;
    }

    PWM_Init_Config_s buzzer_config = {
        .htim = htim,
        .channel = channel,
        .dutyratio = 0,
        .period = 1.0f / 4000.0f, /* 手册: 蜂鸣器典型发声频率 4kHz */
    };
    buzzer = PWMRegister(&buzzer_config);
}

BuzzzerInstance *BuzzerRegister(Buzzer_config_s *config)
{
    if (config->alarm_level > BUZZER_DEVICE_CNT) // 超过最大实例数,考虑增加或查看是否有内存泄漏
        while (1)
            ;
    BuzzzerInstance *buzzer_temp = (BuzzzerInstance *)malloc(sizeof(BuzzzerInstance));
    memset(buzzer_temp, 0, sizeof(BuzzzerInstance));

    buzzer_temp->alarm_level = config->alarm_level;
    buzzer_temp->loudness = config->loudness;
    buzzer_temp->octave = config->octave;
    buzzer_temp->alarm_state = ALARM_OFF;

    buzzer_list[config->alarm_level] = buzzer_temp;
    return buzzer_temp;
}

void AlarmSetStatus(BuzzzerInstance *buzzer, AlarmState_e state)
{
    buzzer->alarm_state = state;
}

void BuzzerTask()
{
    BuzzzerInstance *buzz;
    if (buzzer == NULL)
        return;

    /* 正在播放旋律时不干预 PWM, 避免和旋律抢同一个通道 */
    if (BuzzerIsPlaying())
        return;

    for (size_t i = 0; i < BUZZER_DEVICE_CNT; ++i)
    {
        buzz = buzzer_list[i];
        if (buzz == NULL)
            continue;
        if (buzz->alarm_level > ALARM_LEVEL_LOW)
        {
            continue;
        }
        if (buzz->alarm_state == ALARM_OFF)
        {
            PWMSetDutyRatio(buzzer, 0);
        }
        else
        {
            PWMSetDutyRatio(buzzer, buzz->loudness);
            switch (buzz->octave)
            {
            case OCTAVE_1:
                PWMSetPeriod(buzzer, (float)1 / DoFreq);
                break;
            case OCTAVE_2:
                PWMSetPeriod(buzzer, (float)1 / ReFreq);
                break;
            case OCTAVE_3:
                PWMSetPeriod(buzzer, (float)1 / MiFreq);
                break;
            case OCTAVE_4:
                PWMSetPeriod(buzzer, (float)1 / FaFreq);
                break;
            case OCTAVE_5:
                PWMSetPeriod(buzzer, (float)1 / SoFreq);
                break;
            case OCTAVE_6:
                PWMSetPeriod(buzzer, (float)1 / LaFreq);
                break;
            case OCTAVE_7:
                PWMSetPeriod(buzzer, (float)1 / SiFreq);
                break;
            default:
                break;
            }
            break;
        }
    }
}

/* ============================ 简单旋律播放 ============================ */

static const BuzzerNote_t *music_notes = NULL; /* 当前旋律 */
static uint16_t music_count = 0;               /* 音符总数 */
static uint16_t music_index = 0;               /* 当前音符下标 */
static float music_note_start = 0.0f;          /* 当前音符开始时刻(DWT ms) */
static uint16_t music_duration = 0;            /* 当前音符时长(ms) */
static uint8_t music_playing = 0;              /* 是否在播放 */
static uint8_t music_loop = 0;                 /* 是否循环 */
static uint8_t music_gate_off = 0;             /* 当前音符是否已进入"收音"阶段 */
static float music_tempo = 1.0f;               /* 速度倍率: >1 更快, <1 更慢 */

/**
 * @brief 设置一个音高; freq=0 表示休止(静音)
 */
static void BuzzerSetTone(uint16_t freq_hz)
{
    if (buzzer == NULL)
        return;

    if (freq_hz == 0)
    {
        PWMSetDutyRatio(buzzer, 0.0f);
        return;
    }

    /* 周期 = 1/f, 频率整体上移让音符靠近蜂鸣器谐振点 */
    float f = (float)freq_hz * (float)BUZZER_MUSIC_TRANSPOSE;
    PWMSetPeriod(buzzer, 1.0f / f);
    PWMSetDutyRatio(buzzer, 0.5f);
}

void BuzzerPlayMelody(const BuzzerNote_t *notes, uint16_t count, uint8_t loop)
{
    if (buzzer == NULL || notes == NULL || count == 0)
        return;

    music_notes = notes;
    music_count = count;
    music_index = 0;
    music_loop = loop ? 1 : 0;
    music_playing = 1;
    music_gate_off = 0;
    music_note_start = DWT_GetTimeline_ms();
    music_duration = notes[0].duration_ms;

    BuzzerSetTone(notes[0].freq_hz);
}

void BuzzerSetMelodyTempo(float scale)
{
    if (scale < 0.25f)
        scale = 0.25f;
    else if (scale > 4.0f)
        scale = 4.0f;

    music_tempo = scale;
}

void BuzzerStopMelody(void)
{
    music_playing = 0;
    music_notes = NULL;
    music_count = 0;

    if (buzzer != NULL)
        PWMSetDutyRatio(buzzer, 0.0f);
}

void BuzzerMusicTask(void)
{
    if (!music_playing || music_notes == NULL || buzzer == NULL)
        return;

    /* 音符实际时长 = 谱面时长 / 速度倍率 */
    float dur = (float)music_duration / music_tempo;
    float elapsed = DWT_GetTimeline_ms() - music_note_start;

    /* 收音(articulation): 每个音符的最后 12% 静音,
     * 这样"同音重复"才听得出来是两下, 而不是连成一声长音。
     * 没有这一步, 连续相同音高的旋律听起来就是糊的。 */
    if ((music_gate_off == 0) && (elapsed >= dur * 0.88f))
    {
        music_gate_off = 1;
        if (buzzer != NULL)
            PWMSetDutyRatio(buzzer, 0.0f);
    }

    /* 当前音符还没播完 */
    if (elapsed < dur)
        return;

    music_note_start = DWT_GetTimeline_ms();
    music_gate_off = 0;
    music_index++;

    if (music_index >= music_count)
    {
        if (music_loop)
            music_index = 0;
        else
        {
            BuzzerStopMelody();
            return;
        }
    }

    music_duration = music_notes[music_index].duration_ms;
    BuzzerSetTone(music_notes[music_index].freq_hz);
}

uint8_t BuzzerIsPlaying(void)
{
    return music_playing;
}

uint8_t BuzzerIsReady(void)
{
    return (buzzer != NULL) ? 1 : 0;
}
