# -*- coding: utf-8 -*-
"""把接线拆成两张极简图：上半只讲电源，下半只讲 CAN。"""
import os

from PIL import Image, ImageDraw, ImageFont

W, H = 1800, 1040
BG = (255, 255, 255)
INK = (25, 33, 46)
MUT = (110, 122, 140)
LINE = (205, 212, 224)
PWR = (214, 48, 49)
CAN = (37, 99, 235)

try:
    F = ImageFont.truetype(r"C:\Windows\Fonts\msyh.ttc", 22)
    FS = ImageFont.truetype(r"C:\Windows\Fonts\msyh.ttc", 18)
    FB = ImageFont.truetype(r"C:\Windows\Fonts\msyhbd.ttc", 27)

except Exception:
    F = FS = FB = ImageFont.load_default()

img = Image.new("RGB", (W, H), BG)
d = ImageDraw.Draw(img)


def txt(x, y, s, font=F, fill=INK, anchor="la"):
    d.text((x, y), s, font=font, fill=fill, anchor=anchor)


def box(x, y, w, h, label, sub=None, col=PWR, fill=(255, 246, 246)):
    d.rounded_rectangle([x, y, x + w, y + h], radius=8, fill=fill,
                        outline=col, width=2)
    if sub:
        txt(x + 16, y + h / 2 - 15, label, F, col)
        txt(x + 16, y + h / 2 + 11, sub, FS, MUT)
    else:
        txt(x + w / 2, y + h / 2, label, F, col, "mm")


def poly(pts, col, w=4):
    d.line([(int(a), int(b)) for a, b in pts], fill=col, width=w, joint="curve")


def dot(x, y, col, r=7):
    d.ellipse([x - r, y - r, x + r, y + r], fill=col)


# ================================================================ 上半：电源
txt(50, 24, "① 电源：一条主干，四个分叉", FB)
txt(50, 62, "从电池出发，问「下一站是谁」，走到头就停", FS, MUT)
d.line([(50, 92), (1750, 92)], fill=LINE, width=2)

box(60, 116, 170, 74, "电池", "24V XT60")
box(60, 224, 170, 74, "电管", "总分配")
poly([(145, 190), (145, 224)], PWR, 5)

TR = 300
poly([(230, 261), (TR, 261)], PWR, 5)
poly([(TR, 261), (TR, 520)], PWR, 5)

rows = [
    (224, "底盘分电板", "4×3508 + C620　|　转接板 → MG4005", "Chassis XT30"),
    (326, "云台分电板", "2×4310　|　稳压 → 小电脑　|　2×3508（摩擦轮）", "Gimbal XT30 → 滑环 ⑤"),
    (428, "超电功率板", "电容管理模块 → 超级电容组", ""),
    (520, "裁判系统主控", "场地交互 / 灯条 / 小装甲 / 测速模块", "5V / 12V"),
]
for y, nm, sub, tag in rows:
    poly([(TR, y + 42), (360, y + 42)], PWR, 4)
    dot(TR, y + 42, PWR)
    box(360, y, 700, 84, nm, sub, PWR, (255, 248, 248))
    if tag:
        txt(1080, y + 30, tag, FS, MUT)

txt(60, 620, "云台分电板不再升降压，只做二次分配；小电脑的电压由稳压模块单独给。", FS, MUT)

# ================================================================ 下半：CAN
d.line([(50, 664), (1750, 664)], fill=LINE, width=2)
txt(50, 686, "② CAN：只有两条线", FB)
txt(50, 724, "每条线两端各一个 120Ω，设备并联挂在中间", FS, MUT)

# 云台侧
y = 800
poly([(250, y), (1700, y)], CAN, 5)
txt(60, y - 14, "云台侧", FB, CAN)
txt(60, y + 16, "不出滑环", FS, MUT)
for x, nm in [(420, "达妙 MC02"), (700, "4310 ×2"), (980, "3508 ×2\n摩擦轮"),
              (1300, "外挂 IMU")]:
    dot(x, y, CAN)
    poly([(x, y), (x, y - 14)], CAN, 4)
    txt(x, y - 16, nm, FS, CAN, "md" if "\n" not in nm else "md")

# 底盘侧
y2 = 950
poly([(250, y2), (1700, y2)], CAN, 5)
txt(60, y2 - 14, "底盘侧", FB, CAN)
txt(60, y2 + 16, "过滑环", FS, MUT)
for x, nm in [(420, "达妙 MC02"), (700, "3508 ×4"), (980, "MG4005"),
              (1300, "网关 C 板")]:
    dot(x, y2, CAN)
    poly([(x, y2), (x, y2 - 14)], CAN, 4)
    txt(x, y2 - 16, nm, FS, CAN, "md")

# 两端终端电阻
for yy in (y, y2):
    for xx in (250, 1700):
        d.rectangle([xx - 16, yy - 9, xx + 16, yy + 9], fill=(255, 255, 255),
                    outline=CAN, width=2)
        txt(xx, yy + 13, "120Ω", FS, CAN, "ma")

# 滑环示意
poly([(1620, y + 6), (1620, y2 - 6)], CAN, 3)
txt(1640, (y + y2) / 2 - 12, "滑环", FS, CAN)
txt(1640, (y + y2) / 2 + 12, "④ / ①", FS, MUT)

img.save(os.path.join(os.path.dirname(os.path.abspath(__file__)), "接线_两张图.png"))
print("saved")
