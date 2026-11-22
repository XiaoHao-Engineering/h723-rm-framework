# 发射热量管理（loader_heat）

`rm_config.h` 里的 `SHOOT_HEAT_*` 参数 + `shoot/loader_heat.c/h`，把视频里那套
「**底层状态机估计（数） + 上层前馈规划（算） + 硬限幅（兜底）**」落到代码里。

默认按 **步兵 17mm / 冷却优先** 配置：`Q0=88`、`a=24/s`、`d=10`
→ 稳态射频 `a/d = 2.4 发/s`，对应拨盘 `12 rpm`（12 槽）。

## 1. 三层结构

```
            ┌─────────────────────────────────────────────┐
            │ 上层 · 前馈规划（算）                        │
            │ c_allow = ((Q0 - Qest)/0.1s + a) / d        │
            │ 稳态优先时再取 min(c_allow, a/d)             │
            └──────────────────▲──────────────────────────┘
                               │ Qest
            ┌──────────────────┴──────────────────────────┐
            │ 底层 · 状态机估计（数）                      │
            │ 摩擦轮电流 > 阈值 持续 20ms → 确认一发 +d    │
            │ 每周期 Qest -= a·dt                          │
            └──────────────────┬──────────────────────────┘
                               │
            ┌──────────────────▼──────────────────────────┐
            │ 兜底 · 硬限幅（限）                          │
            │ Qest + 2d > Q0 → 只许单发                    │
            │ Qest + 1d > Q0 → 全部禁止（否则要降到 0 才解锁）│
            └─────────────────────────────────────────────┘
```

## 2. 参数（`rm_config.h`）

| 宏 | 默认 | 说明 |
|---|---|---|
| `SHOOT_HEAT_ENABLE` | 1 | 总开关（0 时退回固定 `LOADER_FIRE_RPM`） |
| `SHOOT_HEAT_COOLING_FIRST` | **1** | 1=冷却优先(88/24)，0=爆发优先(230/14) |
| `SHOOT_HEAT_LIMIT_COOL` | 88 | 冷却优先热量上限 |
| `SHOOT_HEAT_COOL_RPS_COOL` | 24 | 冷却优先冷却速率 /s |
| `SHOOT_HEAT_LIMIT_BURST` | 230 | 爆发优先热量上限 |
| `SHOOT_HEAT_COOL_RPS_BURST` | 14 | 爆发优先冷却速率 /s |
| `SHOOT_HEAT_PER_SHOT` | 10 | 每发消耗（17mm=10，42mm=100） |
| `SHOOT_HEAT_SETTLE_MS` | 100 | 裁判系统结算周期 |
| `SHOOT_HEAT_MAX_RPS` | 10 | 机械可支持的连发上限（发/s），只做钳位 |
| `SHOOT_HEAT_POLICY_STEADY` | 1 | 1=稳态优先(≈a/d)，0=允许吃满缓冲 |
| `SHOOT_HEAT_DETECT_AMP` | **1.4 A** | 发弹确认的电流阈值（**要实车标定**） |
| `SHOOT_HEAT_DETECT_MS` | 20 | 电流持续多久算一发 |
| `SHOOT_HEAT_DETECT_REARM_RATIO` | 0.6 | 电流回落到阈值 60% 以下才重新武装（防多数） |
| `SHOOT_HEAT_FRICTION_READY_APS` | 12000 | 摩擦轮起转判据（deg/s，约 2000 rpm） |
| `SHOOT_HEAT_CONFIRM_TIMEOUT_MS` | 300 | 下令后没确认 → 按“打了”保守加热量（防漏检超限） |
| `SHOOT_HEAT_SOFT_RATIO` / `HARD_RATIO` | 0.85 / 0.95 | **只用于日志告警**，真正生效的是“再打几发会不会超” |

> 硬限幅不是拍脑袋的百分比：用的是 `Qest + n·d > Q0` 反推。
> 冷却优先下 `d=10` 占 `Q0` 的 11%，所以 `Q>68` 就禁连发、`Q>78` 全禁——比 0.85 更严，这是对的。

## 3. 和 `shoot.c` 的接线

```c
/* ShootInit() */
LoaderHeatInit();

/* ShootTask() 每周期（100Hz） */
LoaderHeatFeedFriction(aps_l, aps_r, current_a);   /* 两轮取较大值 */
LoaderHeatFeedCommand(cmd_rps);                    /* 只有连发才喂 */
LoaderHeatUpdate(now);

/* 连发：射频由规划给出 */
loader_target_rpm = LOADER_DIR * LoaderHeatGetAllowedRps() * 60.0f / LOADER_SLOTS;

/* 单发：下令时记一笔 */
LoaderHeatOnSingleShot();

/* 限幅优先级最高 */
if (LoaderHeatIsBlocked()) loader_target_rpm = 0.0f;
```

电流换算：C620 反馈 `real_current` 是 raw int16，`±16384 ↔ ±20A`，所以
`current_a = real_current * 20 / 16384`（`FrictionCurrentA()`）。

## 4. VOFA+ 波形（UART1）

| 项 | 值 |
|---|---|
| 串口 | **USART1**（`BOARD_USART1`，板上“对外串口”） |
| 波特率 | **115200**（CubeMX 里当前值） |
| 协议 | **JustFloat** |
| 通道 | **8** |
| 发送频率 | **25 Hz**（`SHOOT_HEAT_VOFA_DIV=4`，100Hz/4） |

| 通道 | 含义 | 单位 |
|---|---|---|
| 0 | `Qest` 热量估计 | 点 |
| 1 | `Q0` 热量上限 | 点 |
| 2 | 允许射频 `c_allow` | 发/s |
| 3 | 指令射频 `cmd_rps` | 发/s |
| 4 | 拨盘指令转速 | rpm |
| 5 | 拨盘实测转速 | rpm |
| 6 | 已确认发数 | 发 |
| 7 | 状态位：低 4 位状态机，bit4 只许单发，bit5 全禁 | — |

VOFA+ 里新建 JustFloat 窗口，波特率 115200，8 通道即可。
**注意**：115200 下 36 字节一帧约 3ms 阻塞，所以默认 25Hz；
如果把 USART1 提到 460800/921600（CubeMX 改），可以把 `SHOOT_HEAT_VOFA_DIV` 改成 1。

## 5. 上机标定顺序

1. **摩擦轮起转阈值**：开摩擦轮，看通道 5 之前的状态位是否从 0 变 1；
   若一直 0，把 `SHOOT_HEAT_FRICTION_READY_APS` 调小（比如 8000）。
2. **电流阈值**：单发打 5 发，看通道 6（已确认发数）是否 = 5；
   - 数少了 → `SHOOT_HEAT_DETECT_AMP` 调小（1.2 → 1.0）；
   - 数多了 → 调大（1.6），或把 `REARM_RATIO` 调大；
   - 顺带看日志有没有 `[heat] miss`（漏检兜底触发）。
3. **射频**：连发时通道 2 应该稳在 **2.4**，通道 4 稳在 **12 rpm**，
   通道 0（Qest）应该贴着一个不高的平台（稳态），不应该一直往上爬。
4. **限幅**：强制连打，看 Qest 到 68 后通道 4 归零（禁连发）、到 78 后单发也被拒
   （日志 `[heat] blocked, single shot refused`）。
5. **切换爆发优先**：`SHOOT_HEAT_COOLING_FIRST` 改 0，重烧，
   此时上限 230 / 冷却 14 → 稳态 1.4 发/s（约 7 rpm）。

## 6. 已知限制

| 限制 | 说明 | 缓解 |
|---|---|---|
| 漏检 | 电流脉冲没抓到 → Qest 偏低 → 可能超限 | `CONFIRM_TIMEOUT_MS` 兜底按“打了”加热量，宁可高估 |
| 多数 | 一个长脉冲被数成两发 | `REARM_RATIO` 要求电流先回落 |
| 低速卡弹 | 连发只有 12 rpm，速度比判据不可靠 | `LOADER_JAM_MIN_RPM` 已从 8 提到 30，低速交给扭矩保护 |
| 冷却增益 | 隧道 1.5× / 战亡清零 | 裁判系统没接，暂用固定 24/s；接上后改 `LoaderHeat` 里的 `lh_cool` 即可 |
| 弹种 | 目前按 17mm 的 d=10 | 换 42mm 改 `SHOOT_HEAT_PER_SHOT=100` |

## 7. 想改？

```powershell
# 改参数只动 rm_config.h，然后重烧
cmake --build build/Debug
openocd -f openocd_dap_faed.cfg -c "program build/Debug/RM.elf verify reset exit"
```

VOFA 抓数据时建议先把 `SHOOT_HEAT_VOFA_DIV` 改成 2（50Hz），波形更顺，代价是 6ms/帧。

## 8. 台架调试（没接摩擦轮 / 没接遥控）

没接摩擦轮时，热量状态机确认不了发弹；没接遥控时 `robot_cmd` 会把 `shoot_mode` 置 OFF，
拨盘根本不使能。所以加了两个**台架开关**（都在 `rm_config.h`）：

| 宏 | 台架值 | 作用 | 上场前 |
|---|---|---|---|
| `RM_BENCH_SHOOT_ENABLE` | **0**（有遥控） | 不看遥控：`shoot_mode` 强制 ON、摩擦轮强制关；**按键(PA15)循环切换 停 → 逐发 → 连发** | 改 **0** |
| `SHOOT_HEAT_BENCH_BYPASS` | **1**（没摩擦轮） | 热量限幅旁路：连发按机械速率 `LOADER_FIRE_RATE_HZ=10 发/s`（50 rpm），不做 a/d 限制、不拦单发 | 改 **0** |

台架下的行为：

```
ShootInit  -> 拨盘使能(位置/速度模式), 摩擦轮不给输出(没接线也不报错)
按键 1 次  -> [bench] load_mode -> 1 (1-BULLET)   一发一发
按键 1 次  -> [bench] load_mode -> 2 (BURST)      10 发/s 连发
按键 1 次  -> [bench] load_mode -> 0 (STOP)       停(当急停用)
```

> 估算(VOFA 通道 0/6)在台架下仍然工作，只是没有摩擦轮电流 → 确认不到发弹、Qest 一直是 0。
> 等接了摩擦轮，把 `SHOOT_HEAT_BENCH_BYPASS` 改 0，就能看到真实的 Qest 曲线。

### 台架最小接线

| 接什么 | 要求 |
|---|---|
| 拨弹盘 | **CAN3**、电机 ID = **1**（`LOADER_CAN_ID`）、24V 供电、CANH/CANL、共地 |
| 摩擦轮 | **可以不接**（台架模式下强制关闭输出） |
| 遥控 | **可以不接**（台架模式不检查离线） |
| 调试串口 | USART1 → USB-TTL，115200，VOFA+ JustFloat 8 通道（可选） |

### 台架验证顺序

1. 通电，看日志有没有 `[dm_motor] motor id [1] mode [3] registered` 和 `[shoot] loader enabled, ready`；
2. 按一下按键 → `[bench] load_mode -> 1`，拨盘应**一发一发**转（约 60 rpm，一次一个弹距）；
3. 再按一下 → `[bench] load_mode -> 2`，拨盘**连续转**（50 rpm ≈ 10 发/s）；
4. 再按一下 → `[bench] load_mode -> 0`，停；
5. 中途**手动捏住拨盘**：应能看到 `[shoot] jam detected, retry 1/3` → 反转退让；
6. 连捏三次：`[shoot] jam x3 -> backoff 60 deg`，拨盘回拨后继续可用，**不失能**。

## 9. CAN ID 规划（重要）

| 设备 | 帧 | 说明 |
|---|---|---|
| 拨弹盘 DM4310（`LOADER_CAN_ID=1`） | 控制 **0x201**（速度模式） | 反馈帧 = 电机里的 **Master ID**，出厂 **0** |
| 摩擦轮 M3508 #1/#2 | 反馈 **0x203 / 0x204** | `FRICTION_L_ID=3`、`FRICTION_R_ID=4`，**C620 电调也要设成 3 / 4** |

为什么摩擦轮不能用 1 / 2：

> 达妙速度模式的控制帧是 **0x200 + ID = 0x201**，而 M3508 #1 的反馈帧也是 **0x201**。
> 两者同时存在时，DJI 驱动会把达妙的控制帧当成 M3508 反馈解析 → 摩擦轮反馈全乱。
> 所以摩擦轮挪到 3 / 4（反馈 0x203 / 0x204），与达妙不冲突。

为什么拨盘注册 `rx_id = 0`：

> 达妙反馈帧 ID 由上位机里的 **Master ID** 决定（手册：默认 0）。
> 代码里 `LOADER_RX_ID` 必须等于它，否则一帧都收不到（日志 `[dm_motor] Motor lost`）。
> 当前默认 0；若你的电机 Master ID 被改成了 1，把 `LOADER_RX_ID` 改 1 即可。

另外：`dm_motor.c` 里 `can_init_config.rx_id` 的语义是
**`DM_RX_ID_AUTO(0xFF)` = 自动取 motor_id，其它值原样使用**。
云台的达妙电机用的是 `DM_RX_ID_AUTO`（保持原来的自动行为），拨盘显式给 0。
