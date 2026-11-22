# h723-rm-framework

> **把 RoboMaster 控制框架，完整搬到 STM32H723 + 达妙 DM-MC-Board02 上。**
> 框架结构保持原样，底层从 F4/bxCAN 全部换成 H7/FDCAN，并补齐了达妙电机、DBUS 遥控、摩擦轮/拨盘控制与一整套安全保护。

![platform](https://img.shields.io/badge/STM32-H723VGT6-03234B?logo=stmicroelectronics&logoColor=white)
![core](https://img.shields.io/badge/Cortex--M7-275MHz-red)
![rtos](https://img.shields.io/badge/FreeRTOS-10.x%20%2B%20CMSIS--RTOS%20v2-2C8EBB)
![can](https://img.shields.io/badge/FDCAN-3%C3%971Mbps-005571)
![build](https://img.shields.io/badge/build-0%20warning%20%2F%200%20error-brightgreen)
![license](https://img.shields.io/badge/license-MIT-blue)

---

## 这个仓库能给你什么 ⭐

不是"换个芯片能跑就行"的移植，而是**一套已经在真实机构上跑通的 H7 + 达妙平台底座**：
三路 FDCAN、达妙/大疆电机、遥控解析、摩擦轮与拨盘控制、失控保护，**接线就能用**。

### 拿到手就能用的能力

| 能力 | 状态 | 关键点 |
|---|---|---|
| 三路 FDCAN（1 Mbps 经典帧） | ✅ 实测 | 底盘 / 云台 / 发射独立总线，抽象成 `BOARD_CAN1/2/3` |
| 达妙 DM4310（MIT / 速度 / 位置速度） | ✅ 实测 | 一拖四、Master ID 自适应、模式切换带时序 |
| 大疆 M3508 / M2006 | ✅ 实测 | 速度环 + 电流环，C620 / C610 通用 |
| DBUS / SBUS 遥控 | ✅ 实测 | 100k 8E1，滑窗自对齐，掉线守护 |
| 摩擦轮控制 | ✅ 实测 | 初速反解 + 斜坡 + 刹车 + 失控看门狗 |
| 拨盘单发 / 连发 | ✅ 实测 | 角度闭环 + 卡弹处理 + 冷却 |
| VOFA+ 8 通道遥测 | ✅ 实测 | JustFloat，115200 下约 22 Hz |
| HardFault 现场转储 | ✅ | 寄存器 / 栈帧自动打印 |

### 几个能量化的数字

| 指标 | 数值 |
|---|---|
| 编译 | **0 warning / 0 error**（GCC 15.2.1 + CMake + Ninja） |
| Flash | **175 KB / 1 MB**（含 FreeRTOS + 全部模块） |
| 任务周期 | 电机 1 kHz · 机器人 100 Hz · IMU 1 kHz |
| FDCAN | 3 路 × 1 Mbps，满负荷无丢帧 |
| 仓库体积 | **17 MB**（已剔除 854 MB 构建产物与 344 MB 演示视频） |
| 上手时间 | clone → 编译 → 烧录 **< 5 分钟** |

### 谁适合 Star

- 正在把 **F4 框架迁到 H7**，尤其被 **D-Cache / DMA 一致性**坑过的人
- 用 **达妙 DM4310** 做云台或拨盘，卡在"使能帧 ID / Master ID / 模式切换"的人
- 做**摩擦轮 / 拨盘**，被"漏发、双发、满电流失控"折磨过的人
- 想要一个**能直接跑**的 H7 + FreeRTOS + FDCAN 底座，而不是又一个点灯例程

### 不适合

- 想要**现成整车参数 / 比赛战术**的（这里是框架与驱动，不是策略库）
- 用的不是**达妙 MC02** 板（`bsp` 层引脚需要自己改）

> 一句话：**这里不只是能跑的代码，还是"为什么这样写"的答案。**

---

## 项目概览

```
┌──────────────────────────── application ────────────────────────────┐
│  cmd(遥控/键鼠)   gimbal(三轴)   shoot(摩擦轮+拨盘)   chassis        │
└───────────────▲──────────────────────▲──────────────────────────────┘
                │  pub / sub (message_center)
┌───────────────┴──────────────────────┴──────────────────────────────┐
│  modules: motor(DJI/DM/HT/LK) · remote(DBUS) · imu(BMI088) ·        │
│           daemon · alarm · vofa · can_comm · ld(日志)               │
└───────────────▲─────────────────────────────────────────────────────┘
┌───────────────┴─────────────────────────────────────────────────────┐
│  bsp: can(FDCAN×3) · usart(DMA+IDLE) · spi · iic · pwm · adc · dwt  │
│       ws2812 · flash · usb · fault(崩溃现场) · board(电源/按键)      │
└───────────────▲─────────────────────────────────────────────────────┘
       STM32H723VGT6  ·  CubeMX(Core/)  ·  FreeRTOS 10.x
```

---

## 硬件平台

| 项 | 说明 |
|---|---|
| 主控 | **STM32H723VGT6**，Cortex-M7 @275 MHz（VOS0） |
| 开发板 | **达妙 DM-MC-Board02**（板载 BMI088、WS2812、蜂鸣器、5V 可控输出、三路 CAN） |
| CAN1 | PD0/PD1（底盘 M3508） |
| CAN2 | PB5/PB6（云台达妙电机） |
| CAN3 | PD12/PD13（发射：DM4310 拨盘 + 2×M3508 摩擦轮） |
| 遥控 | UART5 / PD2，DBUS 100k 8E1（板载反相） |
| 遥测 | USART1 / PA9 (115200) → VOFA+ JustFloat，8 通道 |

---

## 快速开始

### 1. 依赖

| 工具 | 版本 |
|---|---|
| arm-none-eabi-gcc | ≥ 13（已验证 15.2.1） |
| CMake | ≥ 3.22 + Ninja |
| OpenOCD | ≥ 0.12（DAPLink / ST-Link） |

### 2. 编译

```bash
cmake --preset Debug          # 首次配置
cmake --build --preset Debug  # 输出 build/Debug/RM.elf
```

启用 RoboMaster 应用层（cmd/gimbal/shoot/chassis）：

```bash
cmake --preset Debug -DUSE_RM_EXAMPLE=ON
cmake --build --preset Debug
```

> 两种配置都是 **0 warning / 0 error**。

### 3. 烧录

```bash
openocd -f openocd_dap_faed.cfg -c "init" \
        -c "program build/Debug/RM.elf verify reset exit"
```

### 4. 看日志

* **RTT**（需 DAPLink/J-Link）：`SEGGER_RTT` 已把上行缓冲调到 8 KB；
* **串口**：USART1 115200-8N1 输出文本日志。

### 5. 看波形（VOFA+）

USART1，**JustFloat**，8 通道：

| 通道 | 内容 |
|---|---|
| 0/1 | 左/右摩擦轮实测转速（度/秒） |
| 2/3 | 左/右摩擦轮电流（A） |
| 4 | 摩擦轮指令转速（含方向） |
| 5/6 | 拨盘实测 / 指令 rpm |
| 7 | 状态位（热量状态 / 只允许单发 / 已锁定） |

---

## 目录结构

```
.
├── Core/                 CubeMX 生成代码（时钟/GPIO/FDCAN/SPI/UART/ADC/USB）
├── bsp/                  板级支持包（can/usart/spi/pwm/adc/dwt/ws2812/fault/board）
├── modules/              motor(DJI/DM) · remote(DBUS) · imu · daemon · vofa · message_center
├── application/
│   ├── robot.c           总入口 RobotInit() / RobotTask()
│   ├── rm_example/       cmd / gimbal / shoot / chassis（RoboMaster 应用层）
│   └── demo/             开箱自检 app（IMU/灯/蜂鸣器/电压）
├── docs/                 移植说明、踩坑记录、实验手册
├── tools/                irq_audit.py / ioc_audit.py / 简谱转 C 等
└── CMakeLists.txt
```

---

## 核心实现细节（挑三个最能说明问题的）

### ① H7 D-Cache 与串口 DMA 一致性问题

```c
/* bsp/usart/bsp_usart.c —— 解析前必须失效对应缓存行 */
static void USART_RxCacheInvalidate(void *addr, uint32_t size)
{
    if ((SCB->CCR & SCB_CCR_DC_Msk) == 0U) return;
    uint32_t start = ((uint32_t)addr) & ~31U;
    uint32_t end   = ((uint32_t)addr + size + 31U) & ~31U;
    for (uint32_t a = start; a < end; a += 32U)
        SCB->DCIMVAC = a;      /* 直接写维护寄存器，绕开 __DCACHE_PRESENT=0 */
    __DSB(); __ISB();
}
```

> 并且**删掉了接收后的 `memset`**：它会把缓存行写脏，回写时正好覆盖 DMA 刚落进来的数据。

### ② 达妙 DM4310 多帧 ID 接收（Master ID 不再困扰）

```c
/* 反馈帧 ID = 电机里的 Master ID（出厂 0），不一定等于 CAN ID */
DMMotorAttachAlias(loader, 0);
DMMotorAttachAlias(loader, LOADER_CAN_ID);
DMMotorAttachAlias(loader, 0x100u + LOADER_CAN_ID);
DMMotorAttachAlias(loader, 0x200u + LOADER_CAN_ID);
```

### ③ 失控看门狗

```c
/* 任一判据持续 300ms -> 强制 STOP + 锁存（需显式 Stop 才能复位） */
if (ref≈0 && |current_cmd| ≥ 0.9·MaxOut)                 -> FORCE STOP
if (|ref|>500 && |speed|>1000 && ref·speed < 0)          -> FORCE STOP
```

---

## 关键配置（`application/rm_example/rm_config.h`）

| 宏 | 作用 |
|---|---|
| `RM_CAN_SHOOT` | 发射总线（默认 CAN3） |
| `LOADER_CAN_ID` / `LOADER_RX_ID` | 拨盘 DM4310 的 CAN ID / Master ID |
| `FRICTION_L_ID` / `FRICTION_R_ID` | 左右摩擦轮 C620 ID |
| `FRICTION_BULLET_SPEED_MPS` | 目标初速（默认 20 m/s，规则 15~25） |
| `FRICTION_WHEEL_RADIUS_M` / `FRICTION_SLIP` | 轮半径与打滑系数（决定反解转速） |
| `FRICTION_RAMP_MS` / `FRICTION_BRAKE_MS` | 起转斜坡 / 刹车时长 |
| `RM_FRICTION_WHEEL_ENABLE` | 是否已接摩擦轮（未接则不发使能） |
| `MINI_PITCH_KP` / `MINI_PITCH_KD` | 小 Pitch 的 MIT 内部环刚度/阻尼 |
| `GRAV_FF_*` | 俯仰重力前馈（质量×力臂×cosθ） |

---

## 踩坑记录（都是真金白银的教训）

> 这一节是整份文档里最省时间的地方：下面每一条都在实机上复现过、定位过、修好了。

| 现象 | 根因 | 解决 |
|---|---|---|
| 串口 DMA 收到全 0 | D-Cache 未失效 + `memset` 回写覆盖 | 解析前按 32 字节失效缓存行（直接写 `DCIMVAC`）；去掉 `memset` |
| 中断里打日志后 HardFault | ISR 里调 `printf` 爆栈 | ISR 只记录变量，任务里打印 |
| 电机满电流不停 | 丢反馈后速度环饱和，C620 保持最后电流 | 丢反馈零电流 + 失控看门狗 + 显式发 0 |
| CAN 一直 ACK 错误（TEC=128） | CAN_H/CAN_L 接反 | 对调后 TEC 归零 |
| 单发打两发 / 漏发 | 时间配额式单发 + 惯性过冲 | 位置误差判到位（1.5°）+ 冷却 250ms + 起转斜坡 |
| 绿灯频闪、不受控 | 模式切换被 100Hz 重入 | 状态机（失能→5ms→写模式→5ms→使能）+ 防重入 |
| 达妙使能帧发了没反应 | 使能帧 ID 随控制模式变 | MIT=ID / 位置=0x100+ID / 速度=0x200+ID，三种都发 |
| 反馈帧收不到 | 反馈 ID = 电机内 Master ID，≠ CAN ID | 挂 `0 / ID / 0x100+ID / 0x200+ID` 多别名接收 |
| DBUS 帧边界漂移 | 协议无帧头，空闲中断切帧不齐 | 滑窗自对齐（开关∈1..3 且通道∈100..1900） |
| 摩擦轮"后半程掉速" | 静差 + 输出饱和 | 加 Ki / 积分限幅；VOFA 看 cmd vs 实测再定 |
| RTT 日志里浮点是空的 | SEGGER printf 不支持 `%f` | 改整数（如 mA）打印 |

---

## 已验证 / 待验证

**已验证**：三路 FDCAN 通信；DM4310 使能/速度/MIT；DBUS 遥控解析（含 H/L、帧对齐）；摩擦轮速度环（斜坡/刹车/看门狗）；拨盘单发角度闭环；VOFA 8 通道；0 warning 构建；IRQ/ioc 审计脚本通过。

**待验证**：6000 rpm 长时间稳态与初速标定（需要测速仪）；小 Pitch 的 MIT 参数手感整定；裁判系统链路。

---

## 致谢

- **湖南大学跃鹿战队（HNU YueLu）** —— 本工程的框架结构、分层方式与命名规范来自其开源的 RoboMaster 基础框架（`basic_framework`）。没有他们的开源，就没有这个移植工程。**请优先 Star 原仓库。**
- **达妙科技** —— DM-MC-Board02 开发板资料、DM4310 电机与驱动协议说明。
- **RoboMaster 社区** 的开源工程（电机驱动写法、摩擦轮/拨盘控制思路等），给了大量参考。

> ⚠️ **版权声明**：**框架版权归原作者（湖南大学跃鹿战队）所有**，本仓库**仅做平台移植与二次开发**，不对原框架主张任何所有权，也不代表原作者立场。若原作者认为本仓库存在不当使用，请联系删除。

## License

- 本仓库中**移植与新增的部分**（H7/FDCAN 底层、达妙电机驱动、DBUS/SBUS 解析、摩擦轮与拨盘控制、README 等）采用 **MIT**；
- **原框架部分的版权归湖南大学跃鹿战队所有**，本仓库仅做平台移植与二次开发。

使用/修改/商用请保留原框架署名与本文档的致谢段落。

---

<p align="center">
如果这个工程帮你少踩了一个坑，欢迎点个 ⭐ Star —— 也欢迎 Issue 交流 H7 / 达妙电机 / 摩擦轮控制的细节。
</p>
