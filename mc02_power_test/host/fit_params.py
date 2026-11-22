# -*- coding: utf-8 -*-
"""
实验一/二的数据拟合：k1、k2、a

用法:
    # 实验一：空载扫转速，拟合 k1、a
    python fit_params.py noload.csv --mode noload

    # 实验二：堵转扫电流，拟合 k2（a 用实验一的值固定）
    python fit_params.py stall.csv --mode stall --a 4.08

    # 两种数据都给，做联合校验
    python fit_params.py noload.csv --mode noload --stall stall.csv

支持两种日志格式:
    --format csv   带表头: t_ms,Icmd,rpm,given,temp,bus_mv,bus_ma
    --format vofa  VOFA FireWater: Icmd,rpm,Pin_mW,bus_mv
"""
import argparse
import csv
import os
import sys

import numpy as np

CT = 1.99688994e-6      # W/(rpm*count)
K1_REF = 1.23e-7
K2_REF = 1.453e-7


def load(path, fmt, fs):
    t, i, w, pin = [], [], [], []
    with open(path, "r", encoding="utf-8", errors="ignore") as f:
        reader = csv.reader(f)
        rows = [r for r in reader if r and not r[0].startswith("#")]
    if not rows:
        sys.exit("空文件")
    if fmt == "csv":
        if rows[0][0].lower().startswith("t_ms"):
            rows = rows[1:]
        for n, r in enumerate(rows):
            try:
                t.append(float(r[0]) / 1000.0)
                i.append(float(r[1]))
                w.append(float(r[2]))
                pin.append(float(r[5]) * float(r[6]) / 1.0e6)
            except (ValueError, IndexError):
                continue
    else:
        for n, r in enumerate(rows):
            try:
                t.append(n / fs)
                i.append(float(r[0]))
                w.append(float(r[1]))
                pin.append(float(r[2]) / 1000.0)
            except (ValueError, IndexError):
                continue
    return (np.array(t), np.array(i), np.array(w), np.array(pin))


def steady_mask(t, w, i, thr=500.0):
    """只保留准稳态点：转速变化率小、电流不太小"""
    if len(t) < 5:
        return np.ones_like(t, dtype=bool)
    dw = np.gradient(w, t)
    return (np.abs(dw) < thr) & (np.abs(i) > 100) & (np.abs(w) > 50)


def linfit(x, y):
    A = np.column_stack([x, np.ones_like(x)])
    coef, *_ = np.linalg.lstsq(A, y, rcond=None)
    k, b = coef
    pred = A @ coef
    ss_res = np.sum((y - pred) ** 2)
    ss_tot = np.sum((y - y.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 0 else 0.0
    rmse = float(np.sqrt(np.mean((y - pred) ** 2)))
    return float(k), float(b), float(r2), rmse


def report(tag, k, b, r2, rmse, n, unit):
    print(f"\n[{tag}]  n={n}")
    print(f"  k = {k:.4e}   ({unit})")
    print(f"  a = {b:.3f} W")
    print(f"  R² = {r2:.4f}   RMSE = {rmse:.3f} W")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--mode", choices=["noload", "stall"], required=True)
    ap.add_argument("--format", choices=["csv", "vofa"], default="csv")
    ap.add_argument("--fs", type=float, default=200.0, help="vofa 日志的采样率")
    ap.add_argument("--a", type=float, default=None, help="已知的 a（实验一结果）")
    ap.add_argument("--k2", type=float, default=None, help="已知的 k2，用于修正 k1")
    ap.add_argument("--stall", default=None, help="堵转数据，用于联合校验")
    ap.add_argument("--plot", default="fit_result.png")
    args = ap.parse_args()

    t, i, w, pin = load(args.file, args.format, args.fs)
    if len(t) < 10:
        sys.exit("有效数据太少")
    print(f"读入 {len(t)} 点，时长 {t[-1]-t[0]:.1f}s，转速 {w.min():.0f}~{w.max():.0f} rpm")

    pm = CT * i * w
    y = pin - pm

    if args.mode == "noload":
        m = steady_mask(t, w, i)
        x = (w[m] ** 2)
        yy = y[m].copy()
        if args.k2 is not None:
            yy -= args.k2 * i[m] ** 2
            print(f"已用 k2={args.k2:.3e} 扣除铜损")
        k1, a, r2, rmse = linfit(x, yy)
        report("空载：Pin - Pm = k1·ω² + a", k1, a, r2, rmse, int(m.sum()), "W/rpm²")
        print(f"  参考值 k1≈{K1_REF:.2e}，a≈4 W；偏差 {abs(k1-K1_REF)/K1_REF*100:.0f}%")
        xlabel, title = "ω² (rpm²)", "实验一：k1 拟合"
        fit_line = k1 * x + a
    else:
        m = (np.abs(w) < 80) & (np.abs(i) > 300)
        if m.sum() < 5:
            m = np.abs(w) < 200
        x = i[m] ** 2
        yy = y[m].copy()
        if args.a is not None:
            yy -= args.a
            k2, _, r2, rmse = linfit(x, yy)
            a = args.a
        else:
            k2, a, r2, rmse = linfit(x, y[m])
        report("堵转：Pin - a = k2·I²", k2, a, r2, rmse, int(m.sum()), "W/count²")
        R = k2 / 2.2e-6
        print(f"  反推相电阻 R ≈ {R*1000:.1f} mΩ（和手册对一下）")
        xlabel, title = "Icmd² (count²)", "实验二：k2 拟合"
        fit_line = k2 * x + a

    # 联合校验
    if args.stall and args.mode == "noload":
        t2, i2, w2, pin2 = load(args.stall, args.format, args.fs)
        m2 = (np.abs(w2) < 80) & (np.abs(i2) > 300)
        y2 = pin2[m2] - CT * i2[m2] * w2[m2]
        k2, _, r22, rmse2 = linfit(i2[m2] ** 2, y2 - a)
        print(f"\n[联合] k2 = {k2:.4e}  R²={r22:.3f}  RMSE={rmse2:.2f} W")
        yt = pin - (CT * i * w + k1 * w ** 2 + k2 * i ** 2 + a)
        print(f"[联合] 整段预测残差：均值 {yt.mean():+.2f} W，RMSE {np.sqrt((yt**2).mean()):.2f} W")

    # 出图
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
        plt.rcParams["axes.unicode_minus"] = False
        fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
        ax[0].scatter(x, yy, s=6, alpha=0.4, label="实测")
        order = np.argsort(x)
        ax[0].plot(x[order], fit_line[order], "r-", lw=1.5, label="拟合")
        ax[0].set_xlabel(xlabel)
        ax[0].set_ylabel("Pin − Pm  (W)")
        ax[0].set_title(title)
        ax[0].legend()
        ax[0].grid(alpha=0.3)
        resid = yy - fit_line
        ax[1].scatter(fit_line, resid, s=6, alpha=0.4)
        ax[1].axhline(0, color="r", ls="--", lw=1)
        ax[1].set_xlabel("拟合值 (W)")
        ax[1].set_ylabel("残差 (W)")
        ax[1].set_title(f"残差  RMSE={rmse:.2f} W")
        ax[1].grid(alpha=0.3)
        fig.tight_layout()
        fig.savefig(os.path.join(os.path.dirname(os.path.abspath(args.file)) or ".",
                                 args.plot), dpi=160)
        print(f"\n图已保存: {args.plot}")
    except Exception as e:
        print("出图失败:", e)

    print("\n贴进代码:")
    if args.mode == "noload":
        print(f"  k1 = {k1:.3e}f;  a = {a:.3f}f;")
    else:
        print(f"  k2 = {k2:.3e}f;  a = {a:.3f}f;")


if __name__ == "__main__":
    main()
