# 实验一：空载扫转速（MC02 + M3508/C620）

> **注意：本目录是"独立小程序"版本的草稿（假设你从 CubeMX 新开一个工程，把 app/ 拷进去）。**
> 现在实验一已经**集成进本工程的框架**了，实际跑的是：
>
> - 固件：`application/power_bench/`（`robot_def.h` 里 `APP_POWER_BENCH = 1` 时生效）
> - 操作手册：`docs/实验一_空载扫转速.md`
> - 上位机：`tools/power_bench_log.py`（录数据）、`tools/power_bench_fit.py`（拟合）
>
> 本目录保留作参考：`c620.c/ina226.c` 的帧编解码和 INA226 驱动以后接传感器时可以直接用，
> `拟合方法.md` 的推导和 Excel 做法也仍然适用。

## 为什么先做实验一

不是因为它最重要，而是因为它是**唯一一个"错了也不会烧东西"的实验**，而且一次能验证四件事：

1. **验证工具链**：CAN 能不能发、0x201 反馈能不能收、1 kHz 定时器准不准、串口能不能稳定吐数据。
   这四件事任何一件有问题，实验二（堵转大电流）就是白做，甚至危险。
2. **拿到 `k1` 和 `a`**：空载时电流很小（`k2·I²` 只有零点几瓦，可以忽略），所以
   `Pin ≈ k1·ω² + a`，对 `ω²` 做一次线性回归就出来了。
3. **给实验二提供 `a`**：堵转拟合 `Pin = k2·I² + a` 时，`a` 已经知道，`k2` 才不会被 `a` 带偏。
4. **摸清电机的"脾气"**：最高转速、启动死区、摩擦大小、发热速度——这些直接决定实验二的脉冲长度和安全电流。
5. **风险最低**：电流 ≤ 2500 counts，轮子悬空，没有堵转、没有大电流、没有机械负载，出问题最多是数据难看。

一句话：**实验一是"校准仪器"，实验二是"称重"，顺序反了，称出来的数不可信。**

---

## 一、硬件清单

| 东西 | 必需 | 说明 |
|---|---|---|
| 达妙 MC02 | ✅ | STM32H723VGT6，FDCAN1 = PD0/PD1 |
| M3508 + C620 | ✅ | C620 的 CAN 接 MC02 的 FDCAN1 |
| 24 V 电源 | ✅ | 建议带电流显示，方便对照 |
| DAPLink | ✅ | 已经插好 |
| INA226 模块 | ⚠️ | **测 `Pin` 必需**；没有的话只能先跑"实验 0"验证 CAN |
| USB-TTL | 可选 | 如果没有，用 MC02 的 USB 虚拟串口 |

### 接线

```
24V+ ──[INA226 采样电阻]── C620 电源+
24V- ──────────────────── C620 电源-
C620 CAN_H ── MC02 FDCAN1_H (PD1)
C620 CAN_L ── MC02 FDCAN1_L (PD0)
两端各并一个 120 Ω（短距离一端也行）
INA226: SDA/SCL 接你选的 I2C（代码里改句柄），VBUS 接 24V+，GND 共地
```

⚠️ 轮子悬空但要**固定好电机座**，2500 counts 的扭矩足够让电机跳起来。

---

## 二、CubeMX 配置清单

| 项目 | 设置 |
|---|---|
| MCU | STM32H723VGTx |
| 时钟 | HSE 24 MHz（板载），PLL1 → 550 MHz，确保 FDCAN 时钟 80 MHz |
| FDCAN1 | PD0=RX, PD1=TX；**Classic CAN**；1 Mbit/s；接受所有帧 |
| TIM6 | 1 kHz 中断（预分频按主频算） |
| USART1 | PA9=TX, PA10=RX, 921600 8N1（或 USB CDC） |
| I2C1/I2C2 | 100 kHz，接 INA226（没有传感器可跳过） |
| GPIO | 一个 LED 做心跳（可选） |

生成工程后，把 `app/` 里的四个文件加进去，按 `port/main_hal_example.c` 把三个钩子接上。

---

## 三、代码结构

```
app/
  power_test.c/.h   实验状态机（与硬件无关）
  c620.c/.h         C620 帧编解码（纯函数）
  ina226.c/.h       INA226 驱动（I2C 读写用函数指针注入）
port/
  main_hal_example.c  STM32 HAL 胶水层（照抄改引脚）
host/
  fit_no_load.py    上位机：读 CSV、拟合 k1/a、画图
```

`power_test` 只依赖三个由你实现的函数：

```c
void     pt_port_can_send(uint32_t id, const uint8_t data[8]); // 发一帧
uint32_t pt_port_now_ms(void);                                  // 毫秒时钟
void     pt_port_log(const char *s);                            // 输出一行 CSV
```

---

## 四、烧录（DAPLink）

任选一种：

```bash
# pyOCD（推荐，CMSIS-DAP 原生支持）
pyocd flash -t stm32h723vg build/xxx.hex

# OpenOCD
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg \
        -c "program build/xxx.elf verify reset exit"
```

Keil 里把 Debug 选 CMSIS-DAP 也行。

---

## 五、跑实验

上电后 2 秒开始，串口会输出：

```
t_ms,Icmd,rpm,given,temp,bus_mv,bus_ma
2000,0,0,0,28,24050,35
2005,12,0,8,28,24048,35
...
```

曲线形如：电流缓慢上升 → 转速跟着上升 → 到 2500 counts 时稳定在某个转速（摩擦平衡）。

**判据**：

- 转速单调上升、没有跳变；
- `given` 跟 `Icmd` 基本一致（差几十 counts 正常，差几百说明 CAN 或 ID 错）；
- 温度缓慢上升，不超过 60°C；
- 同样的 `Icmd`，两次实验的稳态转速差 < 5%。

---

## 六、上位机拟合

```bash
python host/fit_no_load.py log.csv
```

脚本会：

1. 用 `Pin = bus_mv/1000 * bus_ma/1000` 算输入功率；
2. 用 `Pm = CT * Icmd * rpm` 算机械功率；
3. 对 `Pin − Pm = k1·ω² + a` 做线性回归；
4. 打印 `k1`、`a`、R²，并与参考值（k1≈1.2e-7、a≈4 W）对比；
5. 画散点 + 拟合线。

**通过标准**：R² > 0.95，`k1` 在 0.5e-7 ~ 3e-7 之间，`a` 在 2~8 W。

---

## 七、常见问题

| 现象 | 原因 |
|---|---|
| 收不到 0x201 | CAN 没起、终端电阻没接、C620 没供电、波特率不是 1M |
| 转速一直是 0 | C620 ID 不是 1；或发的是 0x1FF（ID 5-8） |
| `given` 和 `Icmd` 差很多 | 电机堵转/过流保护；或帧字节序错 |
| 轮子不转但电流有 | 启动死区，把起始电流从 300 提到 500 |
| `Pin` 是负的 | INA226 采样电阻方向反了，交换两端 |
| `k1` 是负的 | 转速范围太窄，或电流采样没校准（先用万用表对一下） |

---

## 八、下一步

实验一通过后，把 `i_target` 提到 3000、`ramp_ms` 缩到 30000 再跑一遍，确认 `k1` 稳定；
然后进入实验二：**夹住轮子，短脉冲扫电流，拟合 `k2`**（见 `motor_power_notes/单电机验证方案.md`）。
