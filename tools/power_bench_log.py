#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
把 MC02 串口上的实验数据存成 CSV(实验一/实验二通用)。

固件(application/power_bench)每 20ms 往 USART1 打一行纯数字:

    t_ms,Icmd,rpm,given,temp,vbat_mv,step,pin_mw

同时会夹杂 LOGINFO 的提示行(以空格/`[` 开头)、以及 `[bench] step 3/16 ...`。
本脚本只把"以数字开头"的行写进 CSV, 其余当作日志原样打印到屏幕上,
所以你一边看进度一边就知道该读电流表了。

用法:
    python tools/power_bench_log.py -p COM7 -o noload.csv
    python tools/power_bench_log.py -p COM7 -o noload.csv -t 240      # 只录 240s
    python tools/power_bench_log.py -p COM7 -o noload.csv -b 921600   # 改过波特率

按 Ctrl+C 结束, 文件已经逐行 flush, 不会丢。
"""
import argparse
import os
import sys
import time

HEADER = "t_ms,Icmd,rpm,given,temp,vbat_mv,step,pin_mw\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--port", required=True, help="串口号, 例如 COM7")
    ap.add_argument("-b", "--baud", type=int, default=115200, help="默认 115200")
    ap.add_argument("-o", "--out", default="power_bench.csv", help="输出 CSV 路径")
    ap.add_argument("-t", "--seconds", type=float, default=0.0, help="0 = 一直录到 Ctrl+C")
    args = ap.parse_args()

    try:
        import serial
    except ImportError:
        sys.exit("缺 pyserial: pip install pyserial")

    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.2)
    except Exception as e:
        sys.exit(f"打不开 {args.port}: {e}")

    out_path = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)

    rows = 0
    log_lines = 0
    t0 = time.time()
    print(f"[log] {args.port} @{args.baud}  ->  {out_path}")
    print("[log] 现在给板子上电 / 按 PA15 开始实验; Ctrl+C 结束")

    try:
        with open(out_path, "w", encoding="utf-8", newline="") as f:
            f.write(HEADER)
            f.flush()
            while True:
                if args.seconds > 0 and (time.time() - t0) >= args.seconds:
                    break
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode("utf-8", errors="ignore").strip("\r\n").strip()
                if not line:
                    continue

                # 数据行: 第一个字符是数字或 '-' ; 其余都是人看的日志
                if line[0].isdigit() or line[0] == "-":
                    f.write(line + "\n")
                    rows += 1
                    if rows % 100 == 0:
                        f.flush()
                        print(f"\r[log] {rows} 行  ({rows / 50.0:.0f}s @50Hz)   ", end="", flush=True)
                else:
                    # 日志行: 原样显示, 顺便让用户知道现在是第几档
                    log_lines += 1
                    print("\r" + line)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            ser.close()
        except Exception:
            pass

    print(f"\n[log] 结束: {rows} 行数据, {log_lines} 行日志 -> {out_path}")
    if rows == 0:
        print("[log] 一行数据都没有, 检查: 1) 串口/波特率 2) TX/RX 是否交叉 3) 固件是否 APP_POWER_BENCH=1")
    else:
        print(f"[log] 下一步: python tools/power_bench_fit.py {args.out}")


if __name__ == "__main__":
    main()
