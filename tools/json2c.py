#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
json2c.py —— 把旋律数据 JSON 转换成工程里的 BuzzerNote_t 数组

JSON 需要包含 events 数组, 每个事件至少有:
    freq     频率(Hz), 0 表示休止
    durMs    时长(ms)

用法:
    python tools/json2c.py <输入.json> -n array_name -o 输出.h

示例:
    python tools/json2c.py "雨爱_前30秒_旋律数据.json" -n melody_yuai_v2 -o application/demo/melody_yuai_v2.h
"""

import argparse
import json
import os


def load_events(path):
    with open(path, encoding="utf-8-sig") as f:
        data = json.load(f)

    if isinstance(data, dict) and "events" in data:
        events = data["events"]
        meta = {k: v for k, v in data.items() if k != "events"}
    elif isinstance(data, list):
        events = data
        meta = {}
    else:
        raise SystemExit("JSON 结构不认识: 需要 events 数组")

    out = []
    for e in events:
        freq = e.get("freq", 0) or 0
        dur = e.get("durMs")
        if dur is None:
            # 兼容没有 durMs 的情况, 用 start/end 计算
            dur = int(round((float(e.get("end", 0)) - float(e.get("start", 0))) * 1000))
        dur = int(round(float(dur)))
        if dur < 1:
            dur = 1
        name = e.get("name", "")
        out.append((int(round(float(freq))), dur, name))

    return meta, out


def main():
    ap = argparse.ArgumentParser(description="旋律 JSON -> BuzzerNote_t 数组")
    ap.add_argument("json_path")
    ap.add_argument("-n", "--name", default="my_melody", help="数组名")
    ap.add_argument("-o", "--out", help="输出 .h 路径(不填则打印到屏幕)")
    ap.add_argument("--guard", help="头文件的 include guard(默认由数组名生成)")
    args = ap.parse_args()

    meta, notes = load_events(args.json_path)
    guard = args.guard or ("MELODY_" + args.name.upper() + "_H")
    total_ms = sum(d for _, d, _ in notes)
    voiced = [f for f, _, _ in notes if f > 0]
    fmin, fmax = (min(voiced), max(voiced)) if voiced else (0, 0)

    lines = []
    lines.append("/**")
    lines.append(" * @file %s" % os.path.basename(args.out or (args.name + ".h")))
    lines.append(" * @brief 旋律数据表(由 tools/json2c.py 自动生成, 请勿手工编辑)")
    lines.append(" *")
    if meta.get("title"):
        lines.append(" * 曲目: %s" % meta["title"])
    if meta.get("range"):
        lines.append(" * 范围: %s    BPM: %s    拍号: %s    调性: %s"
                     % (meta.get("range"), meta.get("bpm"), meta.get("timeSignature"), meta.get("key")))
    lines.append(" * 事件数: %d (含休止)   总时长: %.2f 秒" % (len(notes), total_ms / 1000.0))
    lines.append(" * 音域: %d ~ %d Hz" % (fmin, fmax))
    lines.append(" *")
    lines.append(" * 播放: BuzzerPlayMelody(%s, ARRAY_LEN(%s), 0);" % (args.name, args.name))
    lines.append(" */")
    lines.append("")
    lines.append("#ifndef %s" % guard)
    lines.append("#define %s" % guard)
    lines.append("")
    lines.append('#include "buzzer.h"')
    lines.append("")
    lines.append("static const BuzzerNote_t %s[] = {" % args.name)

    row = []
    for i, (freq, dur, name) in enumerate(notes, 1):
        tag = "休止" if freq == 0 else name
        row.append("{ %d, %d }, /* %2d %s */" % (freq, dur, i, tag))
        if len(row) == 4:
            lines.append("    " + "".join(row))
            row = []
    if row:
        lines.append("    " + "".join(row))

    lines.append("};")
    lines.append("")
    lines.append("#endif /* %s */" % guard)

    text = "\n".join(lines) + "\n"
    if args.out:
        with open(args.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text)
        print("已生成 %s" % args.out)
        print("  %d 个事件, 总时长 %.2f 秒, 音域 %d~%d Hz" % (len(notes), total_ms / 1000.0, fmin, fmax))
    else:
        print(text)


if __name__ == "__main__":
    main()
