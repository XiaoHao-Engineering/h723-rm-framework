# -*- coding: utf-8 -*-
"""
M3508 + C620 电机功率模型拟合（复现并校核 model_fit/power_model.m）

模型:
    Pin = Pm + k1*w^2 + k2*Icmd^2 + a
    Pm  = CT * Icmd * w          (机械功率, W)
    CT  = KT/9.55 * 20/16384     (力矩电流系数 ≈ 1.9969e-6 W/(rpm*count))

只依赖 numpy + matplotlib，不需要 MATLAB。
用法:
    python fit_power_model.py            # 拟合并出图
    python fit_power_model.py --no-plot  # 只看数字
"""
import os
import sys
import glob
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

# 让图里的中文正常显示（Windows 自带雅黑/黑体）
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

# ---------------- 常量 ----------------
KT = 0.3 * 187 / 3591           # 转子侧转矩常数 N·m/A
CT = KT / 9.55 * 20 / 16384     # 力矩电流系数 ≈ 1.9969e-6
BUS_V = 24.56                   # 母线电压（报告取固定值）
CMD_MAX = 16384                 # C620 力矩电流控制值上限

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = r"C:\Users\xhao0\Desktop\开源工程\Motor-modeling-and-power-control-master\model_fit\datasets"


def load(path):
    """读取 CubeMonitor 导出的 CSV，并对齐、剔除无效行。

    表头: timestamp_current;current;timestamp_give_current;give_current;
          timestamp_motor_chassis[0].given_current;motor_chassis[0].given_current;
          timestamp_motor_chassis[0].speed_rpm;motor_chassis[0].speed_rpm
    genfromtxt 会去掉非法字符，正好得到 motor_chassis0speed_rpm。
    """
    data = np.genfromtxt(path, delimiter=";", names=True, encoding="utf-8")
    names = [n for n in data.dtype.names if not n.startswith("timestamp")]

    def pick(suffix):
        for n in names:
            if n == suffix or n.endswith(suffix):
                return np.asarray(data[n], float)
        raise KeyError(suffix)

    cur, give, speed = pick("current"), pick("give_current"), pick("speed_rpm")
    given = pick("given_current")
    n = min(map(len, (cur, give, given, speed)))
    cur, give, given, speed = (x[:n] for x in (cur, give, given, speed))
    ok = np.isfinite(cur) & np.isfinite(give) & np.isfinite(speed) & np.isfinite(given)
    return cur[ok], give[ok], given[ok], speed[ok]


def fit(cur, give, speed, bus_v=BUS_V):
    """最小二乘拟合 Pother = k1*w^2 + k2*Icmd^2 + a"""
    pin = bus_v * cur
    pm = CT * give * speed
    target = pin - pm
    A = np.column_stack([speed ** 2, give ** 2, np.ones_like(speed)])
    coef, *_ = np.linalg.lstsq(A, target, rcond=None)
    pred = A @ coef
    ss_res = np.sum((target - pred) ** 2)
    ss_tot = np.sum((target - target.mean()) ** 2)
    return pin, pm, target, coef, 1 - ss_res / ss_tot


def main(plot=True):
    files = sorted(glob.glob(os.path.join(SRC, "*.csv")))
    print(f"{'dataset':<18}{'k1(w^2)':>12}{'k2(I^2)':>12}{'a':>9}"
          f"{'R2':>8}{'RMSE/W':>9}{'n':>10}")
    rows = []
    for f in files:
        cur, give, given, speed = load(f)
        pin, pm, target, (k1, k2, a), r2 = fit(cur, give, speed)
        rmse = np.sqrt(np.mean((target - (k1 * speed**2 + k2 * give**2 + a)) ** 2))
        print(f"{os.path.basename(f):<18}{k1:12.3e}{k2:12.3e}{a:9.3f}"
              f"{r2:8.3f}{rmse:9.2f}{len(speed):10d}")
        rows.append(dict(name=os.path.basename(f), k1=k1, k2=k2, a=a, r2=r2,
                         pin=pin, pm=pm, speed=speed, give=give))

    # ---- 校核部署代码里的常量（来自 test4.csv）----
    t4 = next(r for r in rows if r["name"] == "test4.csv")
    y = t4["pin"] - t4["pm"]
    w, g = t4["speed"], t4["give"]
    for tag, (cw, ci) in {
        "as-fitted   w^2:1.230e-7  I^2:1.453e-7": (1.230e-7, 1.453e-7),
        "as-in-code  w^2:1.453e-7  I^2:1.230e-7": (1.453e-7, 1.230e-7),
    }.items():
        pred = cw * w**2 + ci * g**2 + 4.081
        print(f"  [check] {tag}: RMSE={np.sqrt(np.mean((y-pred)**2)):5.3f} W, "
              f"bias={np.mean(y-pred):+5.3f} W")

    if not plot:
        return

    # ---- 三张图：拟合效果 / 功率构成 / 恒功率解 ----
    fig, ax = plt.subplots(1, 3, figsize=(15.5, 4.3))

    m = (np.abs(w) > 50) & (np.abs(g) > 200)
    pred = (1.230e-7 * w**2 + 1.453e-7 * g**2 + 4.081 + CT * g * w)[m]
    ax[0].scatter(t4["pin"][m], pred, s=5, alpha=0.3)
    lim = [0, max(t4["pin"][m].max(), pred.max()) * 1.02]
    ax[0].plot(lim, lim, "r--", lw=1, label="y = x")
    ax[0].set_xlabel("实测输入功率 (W)")
    ax[0].set_ylabel("模型预测 (W)")
    ax[0].set_title("拟合效果 (test4.csv, RMSE≈0.5 W)")
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)

    I0 = 5000
    ww = np.linspace(0, 9000, 300)
    parts = {
        "机械功率 Pm = CT*I*w": CT * I0 * ww,
        "铜损/力矩相关 k2*I^2": np.full_like(ww, 1.453e-7 * I0**2),
        "铁损/阻尼 k1*w^2": 1.230e-7 * ww**2,
        "静态损耗 a": np.full_like(ww, 4.081),
    }
    for k, v in parts.items():
        ax[1].plot(ww, v, label=k)
    ax[1].plot(ww, sum(parts.values()), "k--", lw=1.2, label="总输入功率")
    ax[1].set_xlabel("转速 (rpm)")
    ax[1].set_ylabel("功率 (W)")
    ax[1].set_title(f"功率构成 @ Icmd={I0}")
    ax[1].legend(fontsize=7)
    ax[1].grid(alpha=0.3)

    P = 45.0
    b = CT * ww
    c = 1.230e-7 * ww**2 + 4.081 - P
    disc = b**2 - 4 * 1.453e-7 * c
    ok = disc >= 0
    rp = np.where(ok, (-b + np.sqrt(np.maximum(disc, 0))) / (2 * 1.453e-7), np.nan)
    rn = np.where(ok, (-b - np.sqrt(np.maximum(disc, 0))) / (2 * 1.453e-7), np.nan)
    ax[2].plot(ww[ok], rp[ok], label="正根（加速，消耗功率）")
    ax[2].plot(ww[ok], rn[ok], label="负根（减速，反电动势）")
    ax[2].axhline(CMD_MAX, c="gray", ls=":", lw=1)
    ax[2].axhline(-CMD_MAX, c="gray", ls=":", lw=1)
    ax[2].axhspan(-CMD_MAX, CMD_MAX, color="green", alpha=0.06)
    ax[2].set_ylim(-30000, 30000)
    ax[2].set_xlabel("转速 (rpm)")
    ax[2].set_ylabel("力矩电流指令 Icmd (counts)")
    ax[2].set_title(f"恒功率 {P:.0f} W 的两个解（绿色区可执行）")
    ax[2].legend(fontsize=8)
    ax[2].grid(alpha=0.3)

    fig.tight_layout()
    dst = os.path.join(HERE, "power_model_analysis.png")
    fig.savefig(dst, dpi=160)
    print("\nsaved", dst)


if __name__ == "__main__":
    main(plot="--no-plot" not in sys.argv)
