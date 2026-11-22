#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
irq_audit.py —— 中断风暴静态检查

背景:
    本工程曾出现过一类致命 BUG: `stm32h7xx_it.c` 里生成了空的 `XXX_IRQHandler`,
    但 NVIC 已经被使能。中断触发后处理函数什么都不做 -> 标志位永不清除 ->
    中断立刻重入 -> CPU 100% 在中断里打转(uwTick 不涨、HAL_Delay 永不返回、
    表现为"程序卡死"但没有任何 HardFault)。

这个脚本做三件事:
    1. 扫出所有 HAL_NVIC_EnableIRQ(XXX_IRQn) 的位置(哪些中断被真正打开了);
    2. 扫出 stm32h7xx_it.c 里每个 XXX_IRQHandler 的函数体;
    3. 报告"已使能但处理函数为空/不存在"的中断。

用法:
    python tools/irq_audit.py            # 检查, 有问题返回非 0
    python tools/irq_audit.py -v         # 同时列出所有已使能的中断
"""

import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def collect_enabled_irqs():
    """扫 Core/Src 与 bsp/modules 下所有 HAL_NVIC_EnableIRQ 调用"""
    found = {}
    for folder in ("Core/Src", "bsp", "modules", "application"):
        for dirpath, _, files in os.walk(os.path.join(ROOT, folder)):
            for name in files:
                if not name.endswith(".c"):
                    continue
                path = os.path.join(dirpath, name)
                try:
                    text = open(path, encoding="utf-8", errors="replace").read()
                except OSError:
                    continue
                for m in re.finditer(r"HAL_NVIC_EnableIRQ\s*\(\s*(\w+)\s*\)", text):
                    rel = os.path.relpath(path, ROOT)
                    found.setdefault(m.group(1), []).append(rel)
    return found


def collect_handlers():
    """解析 it.c: 处理函数名 -> 函数体(去掉注释)"""
    path = os.path.join(ROOT, "Core", "Src", "stm32h7xx_it.c")
    text = open(path, encoding="utf-8", errors="replace").read()
    handlers = {}
    for m in re.finditer(r"void\s+(\w+_IRQHandler)\s*\(void\)\s*\{(.*?)\n\}", text, re.S):
        body = re.sub(r"/\*.*?\*/", "", m.group(2), flags=re.S)
        handlers[m.group(1)] = body
    return handlers


def main():
    ap = argparse.ArgumentParser(description="中断风暴静态检查")
    ap.add_argument("-v", "--verbose", action="store_true", help="列出所有已使能中断")
    args = ap.parse_args()

    enabled = collect_enabled_irqs()
    handlers = collect_handlers()

    if args.verbose:
        print("已使能的中断 (%d 个):" % len(enabled))
        for irq in sorted(enabled):
            print("   %-28s 来自 %s" % (irq, ", ".join(sorted(set(enabled[irq])))))
        print("")

    problems = []
    checked = 0

    for irq, sources in sorted(enabled.items()):
        # 除 Cortex 内核异常外, 外设中断的处理函数名 = 中断名去掉 _IRQn + _IRQHandler
        if not irq.endswith("_IRQn"):
            continue
        base = irq[: -len("_IRQn")]
        handler = base + "_IRQHandler"

        checked += 1
        if handler not in handlers:
            problems.append((irq, handler, "处理函数不存在(将落到 Default_Handler 死循环)", sources))
            continue

        body = handlers[handler]
        if "HAL_" not in body:
            problems.append((irq, handler, "处理函数是空的, 没有调用 HAL_xxx_IRQHandler -> 会中断风暴", sources))

    print("检查了 %d 个已使能的外设中断。" % checked)

    if not problems:
        print("通过: 所有已使能中断都有正确的 HAL 转发调用, 不存在中断风暴风险。")
        return 0

    print("\n发现 %d 个问题:" % len(problems))
    for irq, handler, why, sources in problems:
        print("  [%s]" % irq)
        print("     处理函数: %s" % handler)
        print("     问题    : %s" % why)
        print("     使能处  : %s" % ", ".join(sorted(set(sources))))
    return 1


if __name__ == "__main__":
    sys.exit(main())
