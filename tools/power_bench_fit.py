#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实验一(空载扫转速)数据分析 + 拟合 k1 / a。

输入是 tools/power_bench_log.py 存下来的 CSV:

    t_ms,Icmd,rpm,given,temp,vbat_mv,step,pin_mw

流程:
    1. 按 step 分段, 每段只取"后 50%"(已经稳定)求平均 -> 一个 (电流, 转速, 电压) 点
    2. Pin = 母线电压 × 母线电流
       母线电流从哪来: --manual 文件 (每段一个读数) 或 pin_mw 列 (INA226 自动)
    3. y = Pin - CT·I·ω, x = ω², 一元线性回归 -> k1(斜率), a(截距)

用法:
    # 先看看跑出来的数据长什么样(不需要电流表)
    python tools/power_bench_fit.py noload.csv

    # 有电流表: manual.csv 每行 "段号,电流A[,电压V]"
    python tools/power_bench_fit.py noload.csv --manual manual.csv

    # 以后接了 INA226: pin_mw 列有值, 直接
    python tools/power_bench_fit.py noload.csv

manual.csv 示例(段号就是日志里 step 列, 从 0 开始; 电压不写就用板载 ADC 的值):
    seg,I_A,V_V
    0,0.52,24.12
    1,0.78,24.09
    ...

模型(和 modules/power_control 里一致):
    Pin = CT·I·ω + k1·ω² + k2·I² + a
    空载时 k2·I² 很小(零点几瓦), 所以先当它不存在, 用 k1·ω² + a 拟合;
    转速≈0 的那几段顺便给一个 k2 的粗估(不能替代实验二)。
"""
import argparse
import os
import sys

# Windows 控制台默认是 GBK(cp936), 遇到 ²/✅ 这类字符会直接抛 UnicodeEncodeError。
# 这里把编码错误降级成 '?', 保证脚本在任何控制台上都不会因为一个字符崩掉。
try:
    sys.stdout.reconfigure(errors="replace")
except Exception:
    pass

import numpy as np

CT = 1.99688994e-6      # W/(rpm·count)
K1_REF = 1.23e-7        # M3508 参考值 W/rpm²
K2_REF = 1.453e-7       # 参考值 W/count²
COLS = ["t_ms", "Icmd", "rpm", "given", "temp", "vbat_mv", "step", "pin_mw"]


# ----------------------------------------------------------------- 读数据
def load_csv(path):
    rows = []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or not (line[0].isdigit() or line[0] == "-"):
                continue
            p = line.split(",")
            if len(p) < 7:
                continue
            try:
                v = [float(x) for x in p[:8]]
            except ValueError:
                continue
            while len(v) < 8:
                v.append(0.0)
            rows.append(v)
    if not rows:
        sys.exit(f"{path} 里没有数据行(应以数字开头)。是不是录到的是日志?")
    a = np.array(rows)
    return {
        "t": a[:, 0] / 1000.0,
        "icmd": a[:, 1],
        "rpm": a[:, 2],
        "given": a[:, 3],
        "temp": a[:, 4],
        "mv": a[:, 5],
        "step": a[:, 6].astype(int),
        "pin": a[:, 7],
    }


def load_manual(path):
    """segment -> (I_A, V_V or None)"""
    out = {}
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            p = [x.strip() for x in line.split(",")]
            if not p[0].lstrip("-").isdigit():   # 跳过表头
                continue
            seg = int(p[0])
            i_a = float(p[1])
            v_v = float(p[2]) if len(p) > 2 and p[2] else None
            out[seg] = (i_a, v_v)
    return out


# ----------------------------------------------------------------- 分段统计
def segment_stats(d, frac):
    segs = []
    for s in np.unique(d["step"]):
        if s < 0:            # -1 = 还没进入实验
            continue
        m = d["step"] == s
        n = int(m.sum())
        if n < 5:
            continue
        idx = np.where(m)[0]
        keep = idx[int(n * (1.0 - frac)):]      # 只取后 frac 段(稳态)
        segs.append({
            "seg": int(s),
            "n": int(len(keep)),
            "rpm": float(np.mean(d["rpm"][keep])),
            "ICMD": float(np.mean(d["icmd"][keep])),
            "given": float(np.mean(d["given"][keep])),
            "mv": float(np.mean(d["mv"][keep])),
            "temp": float(np.mean(d["temp"][keep])),
            "pin_mw": float(np.mean(d["pin"][keep])),
        })
    return segs


def linfit(x, y):
    A = np.column_stack([x, np.ones_like(x)])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    k, b = coef
    pred = A @ coef
    ss_res = float(np.sum((y - pred) ** 2))
    ss_tot = float(np.sum((y - y.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rmse = float(np.sqrt(np.mean((y - pred) ** 2)))
    return float(k), float(b), r2, rmse, pred


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--manual", default=None, help="每段的母线电流读数 CSV: seg,I_A[,V_V]")
    ap.add_argument("--steady", type=float, default=0.5, help="每段取后百分之多少(默认 0.5)")
    ap.add_argument("--current", choices=["given", "icmd"], default="given",
                    help="拟合用哪个电流: given=C620 实测(推荐), icmd=自己发的指令")
    ap.add_argument("--min-rpm", type=float, default=300.0, help="低于此转速的点不参与 k1 拟合")
    ap.add_argument("--plot", default=None, help="出图路径, 默认 <csv 同目录>/power_fit.png")
    args = ap.parse_args()

    d = load_csv(args.csv)
    print(f"读入 {len(d['t'])} 行, 时长 {d['t'].max() - d['t'].min():.1f}s, "
          f"转速 {d['rpm'].min():.0f} ~ {d['rpm'].max():.0f} rpm")

    segs = segment_stats(d, args.steady)
    if not segs:
        sys.exit("没有找到任何 step 段, 确认实验跑起来了(按了 PA15)并且录到了数据")

    manual = load_manual(args.manual) if args.manual else {}
    use_pin_col = np.any(np.array([s["pin_mw"] for s in segs]) > 0)

    # ---------------- 每段一行 ----------------
    print("\n段   Icmd  given   rpm   vbat(V)  Pin(W)   temp  n")
    print("-" * 62)
    have_power = True
    for s in segs:
        i_used = s["given"] if args.current == "given" else s["ICMD"]
        if use_pin_col:
            s["I_A"] = None
            s["Pin"] = s["pin_mw"] / 1000.0
        elif s["seg"] in manual:
            i_a, v_v = manual[s["seg"]]
            v = v_v if v_v else s["mv"] / 1000.0
            s["I_A"] = i_a
            s["Pin"] = v * i_a
        else:
            s["I_A"] = None
            s["Pin"] = None
            have_power = False
        s["I_use"] = i_used
        pin_s = f"{s['Pin']:6.2f}" if s["Pin"] is not None else "   ---"
        print(f"{s['seg']:3d} {s['ICMD']:6.0f} {s['given']:6.0f} {s['rpm']:6.0f} "
              f"{s['mv'] / 1000.0:7.2f} {pin_s:>7} {s['temp']:5.0f} {s['n']:4d}")

    # ---------------- 没有功率数据: 只做体检 ----------------
    turning = [s for s in segs if abs(s["rpm"]) > 200]
    print("\n=== 体检 ===")
    if turning:
        first = turning[0]
        print(f"起转死区: 大约在 Icmd = {first['ICMD']:.0f} counts 时轮子开始转")
        print(f"最高转速: {max(abs(s['rpm']) for s in segs):.0f} rpm "
              f"(对应 Icmd = {max(segs, key=lambda s: abs(s['rpm']))['ICMD']:.0f})")
        print(f"最高温度: {max(s['temp'] for s in segs):.0f} C")
    else:
        print("轮子基本没转: 电流太小(启动死区) / 电调没使能 / 轮子被东西挡住了")

    if not have_power:
        print("\n!" * 30)
        print("这批数据里没有'输入功率', 所以只能体检, 不能拟合 k1 / a。")
        print("补一个电流读数就行(每段一个数字):")
        print("  1) 万用表 10A 档串在 24V+ 和 C620 之间, 或者看直流电源的电流显示;")
        print("  2) 按上面的段号写一个 manual.csv: 第一行 seg,I_A, 后面每行 '段号,电流'")
        print("  3) 重跑: python tools/power_bench_fit.py %s --manual manual.csv" % args.csv)
        print("!" * 30)
        return

    # ---------------- 拟合 ----------------
    use = [s for s in segs if abs(s["rpm"]) >= args.min_rpm]
    if len(use) < 3:
        sys.exit(f"参与拟合的点只有 {len(use)} 个(转速>= {args.min_rpm:.0f} rpm), 至少 3 个。"
                 f"把电流档位加大或把 --min-rpm 调小")

    w = np.array([abs(s["rpm"]) for s in use])
    i = np.array([s["I_use"] for s in use])
    pin = np.array([s["Pin"] for s in use])

    y = pin - CT * i * w
    x = w ** 2
    k1, a, r2, rmse, pred = linfit(x, y)

    print(f"\n=== k1 / a  (用 {args.current}, {len(use)} 个点) ===")
    print(f"  k1 = {k1:.4e}  W/rpm^2    (参考 {K1_REF:.2e}, 偏差 {(k1 - K1_REF) / K1_REF * 100:+.0f}%)")
    print(f"  a  = {a:.3f}  W            (参考 4 左右, 合理范围 2~8)")
    print(f"  R^2 = {r2:.4f}   RMSE = {rmse:.3f} W")

    ok = (r2 > 0.90) and (0.4e-7 < k1 < 4e-7) and (1.0 < a < 10.0)
    print("  判定:", "[OK] 通过" if ok else "[FAIL] 不通过  (看文档'常见错误'一节)")

    # ---------------- 转速≈0 的段: 粗估 k2 ----------------
    stall = [s for s in segs if abs(s["rpm"]) < 120 and abs(s["I_use"]) > 100]
    if len(stall) >= 3:
        i2 = np.array([s["I_use"] for s in stall])
        y2 = np.array([s["Pin"] for s in stall]) - a
        k2, b2, r22, rmse2, _ = linfit(i2 ** 2, y2)
        if k2 > 0:
            print(f"\n  附: 转速≈0 的 {len(stall)} 段粗估 k2 = {k2:.3e} W/count² "
                  f"(参考 {K2_REF:.2e}, R²={r22:.3f})")
            print(f"      反推相电阻 R ≈ k2/2.2e-6 = {k2 / 2.2e-6 * 1000:.1f} mΩ "
                  f"(M3508 手册是几十 mΩ)")
            print("      ⚠ 这只是粗估: 轮子悬空时低速段转速并不严格为 0, 正式 k2 要做实验二(堵转)")

    # ---------------- 贴进代码 ----------------
    print("\n改 modules/power_control/power_control.c 的默认值(或调用处传参):")
    print(f"  k1 = {k1:.3e}f;   a = {a:.3f}f;")

    # ---------------- 出图 ----------------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False

        out = args.plot or os.path.join(os.path.dirname(os.path.abspath(args.csv)) or ".",
                                        "power_fit.png")
        fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
        ax[0].scatter(x, y, s=40, label="实测(每段稳态均值)")
        order = np.argsort(x)
        ax[0].plot(x[order], pred[order], "r-", lw=1.5, label=f"拟合 k1={k1:.2e}, a={a:.2f}")
        ax[0].set_xlabel("ω² (rpm²)")
        ax[0].set_ylabel("Pin - CT·I·ω  (W)")
        ax[0].set_title("实验一: 损耗 vs 转速平方")
        ax[0].legend()
        ax[0].grid(alpha=0.3)

        resid = y - pred
        ax[1].scatter(pred, resid, s=40)
        ax[1].axhline(0, color="r", ls="--", lw=1)
        ax[1].set_xlabel("拟合值 (W)")
        ax[1].set_ylabel("残差 (W)")
        ax[1].set_title(f"残差  RMSE={rmse:.2f} W")
        ax[1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(out, dpi=160)
        print(f"\n图已保存: {out}")
    except Exception as e:
        print("出图失败:", e)


if __name__ == "__main__":
    main()
