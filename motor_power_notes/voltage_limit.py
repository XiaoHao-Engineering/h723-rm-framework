# -*- coding: utf-8 -*-
"""电压受限：左图看反电动势撞上母线电压，右图看还剩多少电压能推电流。"""
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

KE = 24.0 / 480.0          # V/rpm（输出轴），24 V 空载 480 rpm
W = np.linspace(0, 520, 600)
E = KE * W

fig, ax = plt.subplots(1, 2, figsize=(11.8, 4.5), sharex=True)

# ---- 左：反电动势 vs 母线电压 ----
ax[0].plot(W, E, lw=2.2, color="tab:blue", label="反电动势 E = Ke·ω")
for v, c in [(24, "tab:green"), (22, "tab:orange"), (20, "tab:red")]:
    ax[0].axhline(v, color=c, ls="--", lw=1.2)
    ax[0].text(6, v + 0.35, f"{v} V 母线", color=c, fontsize=8)
    wx = v / KE
    ax[0].plot([wx], [v], "o", color=c, ms=6)
    ax[0].plot([wx, wx], [0, v], color=c, ls=":", lw=0.9)
ax[0].text(250, 5.0, "交点的横坐标 = 该电压下的空载转速\n24 V → 480 rpm\n22 V → 440 rpm\n20 V → 400 rpm",
           fontsize=8.5, bbox=dict(boxstyle="round", fc="white", ec="0.8"))
ax[0].set_ylabel("电压 (V)")
ax[0].set_xlabel("输出轴转速 ω (rpm)")
ax[0].set_title("反电动势随转速上升，撞到哪条线就到头", fontsize=10)
ax[0].set_ylim(0, 27)
ax[0].grid(alpha=0.3)
ax[0].legend(fontsize=8, loc="upper left")

# ---- 右：剩余电压（= 能推电流的电压）----
for v, c in [(24, "tab:green"), (22, "tab:orange"), (20, "tab:red")]:
    head = np.clip(v - E, 0, None)
    ax[1].plot(W, head, lw=1.8, color=c, label=f"{v} V 电池")
    wx = v / KE
    ax[1].plot([wx], [0], "o", color=c, ms=5)
ax[1].axhspan(0, 2, color="gray", alpha=0.12)
ax[1].text(8, 1.0, "剩余电压 < 2 V：几乎推不动电流", fontsize=8, color="gray")
ax[1].annotate("", xy=(300, 9), xytext=(300, 15),
               arrowprops=dict(arrowstyle="<->", lw=1, color="black"))
ax[1].text(305, 11.4, "300 rpm、24 V：还剩 9 V", fontsize=8)
ax[1].set_ylabel("剩余电压 V_bus − E (V)")
ax[1].set_xlabel("输出轴转速 ω (rpm)")
ax[1].set_title("还剩多少电压可以用来推电流", fontsize=10)
ax[1].set_ylim(0, 27)
ax[1].grid(alpha=0.3)
ax[1].legend(fontsize=8, loc="upper right")

fig.tight_layout()
fig.savefig(r"C:\Users\xhao0\Desktop\我的达妙\motor_power_notes\fig_voltage_limit.png",
            dpi=170, bbox_inches="tight")
print("saved")
