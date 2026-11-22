#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
jianpu2c.py —— 把简谱转换成工程里能直接用的 BuzzerNote_t 数组

为什么需要它:
    蜂鸣器播放器(BuzzerPlayMelody)要的是 {频率Hz, 时长ms} 的表。
    手写这张表要自己算频率, 很麻烦; 这个脚本让你直接贴简谱。

用法:
    # 1) 直接在命令行写简谱(空格分隔, | 是小节线, 会被忽略)
    python tools/jianpu2c.py "1 2 3 1 | 3 4 5 - | 0 5:0.5 5:0.5"

    # 2) 从文本文件读(可以按小节换行, 写成一整段)
    python tools/jianpu2c.py -f my_song.txt -n my_song

    # 3) 调整默认一拍的时长(ms), 默认 500ms = 每分钟 120 拍
    python tools/jianpu2c.py -b 400 "1 2 3"

简谱记号:
    1 2 3 4 5 6 7   中音(1 = C4)
    1' 2' ...       高音(右上角一撇), 2' = 高八度; 也可以写两个撇 1''
    1, 2, ...       低音(右下角一逗号), 低八度; 也可以写两个逗号 1,,
    #4  b7           升号 / 降号
    0               休止符
    -               延长前一个音一拍
    ::<拍数>         自定义时值, 例如 5:0.5(半拍) 5:2(两拍) 5:1.5(附点)
    |               小节线, 忽略

输出:
    一段可以直接粘进 C 文件的 static const BuzzerNote_t 数组。

注意:
    这个脚本只做"数学换算", 不含任何曲谱。请用你自己有的简谱来填。
"""

import argparse
import math
import sys

# 简谱 1~7 相对 do 的半音数
DEGREE_SEMITONE = {1: 0, 2: 2, 3: 4, 4: 5, 5: 7, 6: 9, 7: 11}

# 中音 1 = C4, 对应 MIDI 60
BASE_MIDI = 60

NOTE_NAMES = ["C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B"]


def midi_to_freq(midi: int) -> int:
    """MIDI 音高转频率(Hz), 以 A4=440 为基准, 四舍五入到整数"""
    return int(round(440.0 * (2.0 ** ((midi - 69) / 12.0))))


def parse_pitch(token: str):
    """
    解析一个音高记号, 返回 (midi, 是否有效)

    支持: 1~7, 1'~7', 1,~7,, #4, b7, 0(休止)
    """
    t = token.strip()
    if not t:
        return None, False

    if t == "0":
        return 0, True  # 休止

    accidental = 0
    if t[0] in "#b":
        accidental = 1 if t[0] == "#" else -1
        t = t[1:]

    if not t or not t[0].isdigit():
        return None, False

    degree = int(t[0])
    if degree not in DEGREE_SEMITONE:
        return None, False

    rest = t[1:]
    octave_shift = 0

    # 高音: ' 或 ''
    while rest.startswith("'"):
        octave_shift += 1
        rest = rest[1:]
    # 低音: , 或 ,,
    while rest.startswith(","):
        octave_shift -= 1
        rest = rest[1:]

    if rest:
        return None, False

    midi = BASE_MIDI + DEGREE_SEMITONE[degree] + 12 * octave_shift + accidental
    return midi, True


def convert(text: str, beat_ms: int, name: str) -> str:
    tokens = text.replace("|", " ").split()
    notes = []  # [(midi, beats)]
    last_pitch = None

    for tok in tokens:
        # 单独的 "-" 表示延长前一个音一拍
        if tok == "-":
            if not notes:
                continue
            midi, beats = notes[-1]
            notes[-1] = (midi, beats + 1.0)
            continue

        # "音:拍数"
        beats = 1.0
        pitch_text = tok
        if ":" in tok:
            pitch_text, beats_text = tok.split(":", 1)
            try:
                beats = float(beats_text)
            except ValueError:
                raise SystemExit("时值写错了: %s" % tok)

        midi, ok = parse_pitch(pitch_text)
        if not ok:
            raise SystemExit("看不懂这个记号: %s" % tok)

        notes.append((midi, beats))
        last_pitch = midi

    if not notes:
        raise SystemExit("没有解析到任何音符")

    lines = []
    lines.append("/* 由 tools/jianpu2c.py 生成: 一拍 = %d ms, 共 %d 个音符 */" % (beat_ms, len(notes)))
    lines.append("static const BuzzerNote_t %s[] = {" % name)

    row = []
    for midi, beats in notes:
        ms = int(round(beats * beat_ms))
        if ms < 1:
            ms = 1
        freq = 0 if midi == 0 else midi_to_freq(midi)
        comment = "休止" if midi == 0 else NOTE_NAMES[midi % 12]
        row.append("{ %d, %d }, /* %s */" % (freq, ms, comment))
        if len(row) == 4:
            lines.append("    " + " ".join(row))
            row = []
    if row:
        lines.append("    " + " ".join(row))

    lines.append("};")
    lines.append("")
    lines.append("/* 播放: */")
    lines.append("/* BuzzerPlayMelody(%s, ARRAY_LEN(%s), 0); */" % (name, name))
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description="简谱 -> BuzzerNote_t 数组")
    ap.add_argument("jianpu", nargs="?", help="简谱字符串, 例如 \"1 2 3 1 | 3 4 5 -\"")
    ap.add_argument("-f", "--file", help="从文本文件读取简谱")
    ap.add_argument("-b", "--beat", type=int, default=500, help="一拍多少毫秒(默认 500)")
    ap.add_argument("-n", "--name", default="my_melody", help="生成的数组名(默认 my_melody)")
    args = ap.parse_args()

    if args.file:
        with open(args.file, encoding="utf-8") as f:
            text = f.read()
    elif args.jianpu:
        text = args.jianpu
    else:
        ap.print_help()
        sys.exit(1)

    print(convert(text, args.beat, args.name))


if __name__ == "__main__":
    main()
