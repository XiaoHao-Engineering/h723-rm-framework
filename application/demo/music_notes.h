/**
 * @file music_notes.h
 * @brief 写旋律用的"音高/时值"速记宏 + 简谱换算对照表
 *
 * 目的: 让你直接按简谱写旋律, 而不用自己算频率。
 *
 * ---------------------------------------------------------------------------
 * 一、音高命名
 *   本文件用三组宏表示简谱的三个音区(1=C):
 *
 *     低音(数字下面一点)   L1 ~ L7     例如 L5 = 低音sol
 *     中音(不加点)         M1 ~ M7     例如 M1 = do
 *     高音(数字上面一点)   H1 ~ H7     例如 H1' = 高音do
 *
 *     升半音(#)用对应的 S 结尾宏, 例如 M4S = #4(升fa)
 *     休止符用 REST
 *
 *   说明: 蜂鸣器只能发单音, 所以这些宏只描述"音高", 不涉及和弦。
 *
 * ---------------------------------------------------------------------------
 * 二、时值
 *   用一个"拍"作为单位, 由 MELODY_BEAT_MS 决定(默认 500ms = 每分钟 120 拍)。
 *
 *     四分音符  = 1 拍   -> N(M1, 500)
 *     八分音符  = 1/2 拍 -> N(M1, 250)
 *     十六分    = 1/4 拍 -> N(M1, 125)
 *     二分音符  = 2 拍   -> N(M1, 1000)
 *     附点      = 1.5 拍 -> N(M1, 750)
 *
 *   想要整体变快/变慢, 不用改表, 播放时调用 BuzzerSetMelodyTempo(1.2f) 即可。
 *
 * ---------------------------------------------------------------------------
 * 三、用法示例(公版曲子《欢乐颂》前两句, 贝多芬):
 *
 *   static const BuzzerNote_t ode_to_joy[] = {
 *       N(M3, 500), N(M3, 500), N(M4, 500), N(M5, 500),
 *       N(M5, 500), N(M4, 500), N(M3, 500), N(M2, 500),
 *       N(M1, 500), N(M1, 500), N(M2, 500), N(M3, 500),
 *       N(M3, 750), N(M2, 250), N(M2, 1000),
 *   };
 *
 *   BuzzerPlayMelody(ode_to_joy, ARRAY_LEN(ode_to_joy), 0);
 *
 * ---------------------------------------------------------------------------
 * 四、如果你想批量转换简谱
 *   用工程里的 tools/jianpu2c.py:
 *
 *     python tools/jianpu2c.py "1 2 3 1 | 3 4 5 - | 0 5:0.5 5:0.5"
 *
 *   它会直接打印出可以粘进本工程的 C 数组, 支持:
 *     1~7        中音
 *     1'~7'      高音(数字右上角加一撇)
 *     1,~7,      低音(数字右下角加一逗号)
 *     #4 / b7    升号 / 降号
 *     0          休止
 *     -          延长前一拍
 *     音:拍数    自定义时值, 如 5:0.5
 *     |          小节线(忽略)
 */

#ifndef MUSIC_NOTES_H
#define MUSIC_NOTES_H

#include "buzzer.h"

/* 一拍 = 500ms, 即每分钟 120 拍。想改整体速度改这里, 或运行时调 BuzzerSetMelodyTempo() */
#define MELODY_BEAT_MS 500U

/* 写一个音符的简写 */
#define N(freq, ms) { (freq), (ms) }

/* 休止符 */
#define REST 0

/* ---------------- 低音区(简谱数字下面一点) ---------------- */
#define L1 131 /* do */
#define L1S 139
#define L2 147 /* re */
#define L2S 156
#define L3 165 /* mi */
#define L4 175 /* fa */
#define L4S 185
#define L5 196 /* sol */
#define L5S 208
#define L6 220 /* la */
#define L6S 233
#define L7 247 /* si */

/* ---------------- 中音区(简谱不加点, 1 = C4) ---------------- */
#define M1 262 /* do */
#define M1S 277
#define M2 294 /* re */
#define M2S 311
#define M3 330 /* mi */
#define M4 349 /* fa */
#define M4S 370
#define M5 392 /* sol */
#define M5S 415
#define M6 440 /* la */
#define M6S 466
#define M7 494 /* si */

/* ---------------- 高音区(简谱数字上面一点) ---------------- */
#define H1 523 /* do */
#define H1S 554
#define H2 587 /* re */
#define H2S 622
#define H3 659 /* mi */
#define H4 698 /* fa */
#define H4S 740
#define H5 784 /* sol */
#define H5S 831
#define H6 880 /* la */
#define H6S 932
#define H7 988 /* si */

#endif /* MUSIC_NOTES_H */
