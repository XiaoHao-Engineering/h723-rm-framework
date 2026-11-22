# -*- coding: utf-8 -*-
"""两个根的数值例子：Pin(I) 是一条抛物线，P 是一条水平线。"""
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

CT = 1.99688994e-6      # W/(rpm*count)
K1 = 1.230e-7           # 乘 w^2
K2 = 1.453e-7           # 乘 I^2
A = 4.081               # 静态损耗 W
IMAX = 16384


def pin(w, i):
    return CT * i * w + K1 * w * w + K2 * i * i + A


def roots(w, p):
    b = CT * w
    c = K1 * w * w + A - p
    disc = b * b - 4 * K2 * c
    if disc < 0:
        return None
    s = np.sqrt(disc)
    return (-b + s) / (2 * K2), (-b - s) / (2 * K2), disc


print(f"{'w':>6}{'I+':>10}{'I-':>10}{'Pmin':>9}{'I_vertex':>10}")
for w in [0, 500, 1000, 3000, 6000]:
    r = roots(w, 45.0)
    v = -CT * w / (2 * K2)
    pmin = K1 * w * w + A - (CT * w) ** 2 / (4 * K2)
    if r:
        print(f"{w:6d}{r[0]:10.0f}{r[1]:10.0f}{pmin:9.2f}{v:10.0f}")
    else:
        print(f"{w:6d}{'--':>10}{'--':>10}{pmin:9.2f}{v:10.0f}")

print("\n=== example: w=500 rpm, P=20 W ===")
w, p = 500, 20.0
i1, i2, _ = roots(w, p)
for tag, i in [("I+ (motoring)", i1), ("I- (braking)", i2)]:
    pm = CT * i * w
    loss_i = K2 * i * i
    loss_w = K1 * w * w
    print(f"{tag}: I={i:8.1f}  mech={pm:7.3f}  cu={loss_i:7.3f}  "
          f"iron={loss_w:6.3f}  static={A:5.3f}  total={pin(w, i):7.3f} W")

# ---------------- 画图 ----------------
fig, ax = plt.subplots(1, 2, figsize=(13, 4.8), sharey=True)
i = np.linspace(-IMAX, IMAX, 1200)
for k, (w, p, title) in enumerate([
        (500, 20.0, "ω = 500 rpm，P = 20 W：两个解都可用"),
        (3000, 45.0, "ω = 3000 rpm，P = 45 W：负解被 ±16384 挡住")]):
    y = pin(w, i)
    ax[k].plot(i, y, lw=2, label=f"Pin(I) @ {w} rpm")
    ax[k].axhline(p, color="crimson", ls="--", lw=1.2, label=f"P = {p:.0f} W")
    r = roots(w, p)
    if r:
        for val, c, name in [(r[0], "tab:green", "正根"), (r[1], "tab:orange", "负根")]:
            if abs(val) <= IMAX:
                ax[k].plot([val], [p], "o", color=c, ms=7)
                ax[k].annotate(f"{name} {val:.0f}", (val, p),
                               textcoords="offset points", xytext=(6, 14),
                               color=c, fontsize=9)
            else:
                ax[k].annotate(f"{name} {val:.0f}：在量程外，不可执行",
                               xy=(0.03, 0.80), xycoords="axes fraction",
                               color=c, fontsize=9)
    v = -CT * w / (2 * K2)
    pmin = K1 * w * w + A - (CT * w) ** 2 / (4 * K2)
    if -IMAX <= v <= IMAX:
        ax[k].plot([v], [pmin], "kv", ms=6)
        ax[k].annotate(f"抛物线顶点 {pmin:.1f} W", (v, pmin),
                       textcoords="offset points", xytext=(8, -14), fontsize=8)
    else:
        ax[k].annotate(f"顶点在 I={v:.0f}（量程外），Pmin≈{pmin:.1f} W",
                       xy=(0.03, 0.71), xycoords="axes fraction",
                       fontsize=8, color="gray")
    ax[k].axvline(IMAX, color="gray", ls=":", lw=1)
    ax[k].axvline(-IMAX, color="gray", ls=":", lw=1)
    ax[k].set_xlim(-IMAX * 1.05, IMAX * 1.05)
    ax[k].set_xlabel("力矩电流指令 Icmd (counts)")
    ax[k].set_title(title, fontsize=10)
    ax[k].grid(alpha=0.3)
    ax[k].legend(fontsize=8, loc="upper center")
ax[0].set_ylabel("电机输入功率 Pin (W)")
ax[0].set_ylim(-20, 90)
fig.tight_layout()
fig.savefig(r"C:\Users\xhao0\Desktop\我的达妙\motor_power_notes\fig_two_roots_example.png", dpi=165)
print("\nsaved fig_two_roots_example.png")
