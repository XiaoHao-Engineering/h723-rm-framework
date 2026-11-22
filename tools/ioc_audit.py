#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ioc_audit.py —— 检查 RM.ioc 里关键配置是否符合本工程要求

背景:
    CubeMX 的 .ioc 是纯文本(key=value)。很多"配了但没生效"的情况,
    其实一眼就能在 .ioc 里看出来。这个脚本把必须满足的条件固化下来,
    每次改完 CubeMX 跑一遍即可。

用法:
    python tools/ioc_audit.py            # 检查
    python tools/ioc_audit.py -v         # 额外打印所有 FREERTOS.* 键
"""

import argparse
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
IOC = os.path.join(ROOT, "RM.ioc")


def load_ioc(path):
    cfg = {}
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            cfg[k.strip()] = v.strip()
    return cfg


def find_key(cfg, suffix):
    """.ioc 里 FreeRTOS 的键名可能带前缀, 按后缀匹配"""
    for k, v in cfg.items():
        if k == suffix or k.endswith("." + suffix):
            return v
    return None


def to_int(v):
    try:
        return int(str(v).strip())
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args()

    if not os.path.exists(IOC):
        print("找不到 RM.ioc")
        return 1
    cfg = load_ioc(IOC)

    freertos_keys = {k: v for k, v in cfg.items() if k.startswith("FREERTOS.")}
    if args.verbose:
        print("FREERTOS.* 键 (%d 个):" % len(freertos_keys))
        for k in sorted(freertos_keys):
            print("   %s = %s" % (k, freertos_keys[k]))
        print("")

    problems = []
    oks = []
    warns = []

    def check(name, ok, detail=""):
        (oks if ok else problems).append((name, detail))

    def warn(name, detail=""):
        warns.append((name, detail))

    # ---- FreeRTOS 是否启用 ----
    enabled = len(freertos_keys) > 0
    check("FreeRTOS 已在 .ioc 中启用", enabled,
          "" if enabled else "一条 FREERTOS.* 都没有 -> CubeMX 里 FREERTOS 仍是 Disable, 或改了没保存")

    if enabled:
        iface = (find_key(cfg, "Interface")
                 or cfg.get("VP_FREERTOS_VS_CMSIS_V2.Mode")
                 or cfg.get("VP_FREERTOS_VS_CMSIS_V1.Mode")
                 or "")
        check("接口 = CMSIS_V2", "V2" in iface.upper(), "当前: %s" % (iface or "未找到"))

        want = {
            "configMAX_PRIORITIES": 56,
            "configTICK_RATE_HZ": 1000,
            "configTOTAL_HEAP_SIZE": 32768,
            "configMINIMAL_STACK_SIZE": 128,
            "configUSE_TICKLESS_IDLE": 0,
            "configCHECK_FOR_STACK_OVERFLOW": 2,
            "configUSE_MALLOC_FAILED_HOOK": 1,
            "configSUPPORT_STATIC_ALLOCATION": 1,
        }
        for key, expect in want.items():
            raw = find_key(cfg, key)
            got = to_int(raw)
            if got is None:
                warn("%s = %d(未显式写入)" % (key, expect),
                     "GUI 里与默认值相同就不会写进 .ioc; 请确认 GUI 值确实是 %d" % expect)
            else:
                check("%s = %d" % (key, expect), got == expect, "当前: %d" % got)

        # 默认任务: CubeMX 规定至少保留一个任务, 删不掉是正常的。
        # 真正要检查的是 main.c 有没有调用 MX_FREERTOS_Init()。
        tasks = [v for k, v in cfg.items() if k.startswith("FREERTOS.Tasks")]
        joined = " ".join(tasks)
        if "defaultTask" in joined:
            warn("Tasks 列表里仍有 defaultTask(正常)",
                 "CubeMX 不允许删最后一个任务; 只要不调用 MX_FREERTOS_Init(), 它不会被创建")

        main_c = os.path.join(ROOT, "Core", "Src", "main.c")
        if os.path.exists(main_c):
            src = open(main_c, encoding="utf-8", errors="replace").read()
            called = re.search(r"^\s*MX_FREERTOS_Init\s*\(\s*\)\s*;", src, re.M)
            check("main.c 不调用 MX_FREERTOS_Init()", called is None,
                  "有调用 -> 会创建 defaultTask, 请删除该调用(任务由 RobotInit() 创建)")

    # ---- 时基 / 优先级分组 ----
    tb = cfg.get("NVIC.TimeBase", "")
    check("HAL 时基 = TIM2", tb.startswith("TIM2"), "当前: %s" % tb)
    pg = cfg.get("NVIC.PriorityGroup", "")
    check("优先级分组 = GROUP_4", pg == "NVIC_PRIORITYGROUP_4", "当前: %s" % pg)

    # ---- 5V 使能脚初始电平 ----
    p5 = cfg.get("PC15-OSC32_OUT.PinState", "")
    check("PC15(5V EN) 初始为高", p5 == "GPIO_PIN_SET", "当前: %s" % p5)

    # ---- 外设中断优先级(不允许 0~4) ----
    bad_pri = []
    for k, v in cfg.items():
        if k.startswith("NVIC.") and k.endswith("_IRQn"):
            n = to_int(v)
            if n is not None and n < 5 and "TimeBase" not in k:
                bad_pri.append("%s=%d" % (k, n))
    check("外设中断优先级都 >= 5", not bad_pri,
          ("以下中断优先级 <5: " + ", ".join(bad_pri)) if bad_pri else "")

    # ---- 生成后的 FreeRTOSConfig.h(真正的判定依据) ----
    frc = os.path.join(ROOT, "Core", "Inc", "FreeRTOSConfig.h")
    if os.path.exists(frc):
        src = open(frc, encoding="utf-8", errors="replace").read()

        def getdef(name):
            # 兼容 ((TickType_t)1000) / ((size_t)32768) / ( 56 ) 这类写法
            m = re.search(r"#define\s+%s\s+([^\n]*)" % name, src)
            if not m:
                return None
            nums = re.findall(r"\d+", m.group(1))
            return int(nums[-1]) if nums else None

        checks = [
            ("configMAX_PRIORITIES", 56, ">="),
            ("configTICK_RATE_HZ", 1000, "=="),
            ("configTOTAL_HEAP_SIZE", 32768, ">="),
            ("configMINIMAL_STACK_SIZE", 128, "=="),
            ("configUSE_TICKLESS_IDLE", 0, "=="),
            ("configCHECK_FOR_STACK_OVERFLOW", 2, "=="),
            ("configUSE_MALLOC_FAILED_HOOK", 1, "=="),
            ("configSUPPORT_STATIC_ALLOCATION", 1, "=="),
        ]
        for name, expect, op in checks:
            got = getdef(name)
            if got is None:
                warn("FreeRTOSConfig.h: %s 未定义" % name, "建议显式定义")
                continue
            ok = (got >= expect) if op == ">=" else (got == expect)
            check("FreeRTOSConfig.h: %s %s %d" % (name, op, expect), ok, "当前: %d" % got)
    else:
        warn("尚未生成 Core/Inc/FreeRTOSConfig.h", "生成代码后再跑一次可校验内核参数")

    print("通过 %d 项, 告警 %d 项, 问题 %d 项。" % (len(oks), len(warns), len(problems)))
    for name, _ in oks:
        print("  [OK]   %s" % name)
    for name, detail in warns:
        print("  [~~]   %s" % name)
        if detail:
            print("         %s" % detail)
    for name, detail in problems:
        print("  [!!]   %s" % name)
        if detail:
            print("         %s" % detail)

    return 0 if not problems else 1


if __name__ == "__main__":
    sys.exit(main())
