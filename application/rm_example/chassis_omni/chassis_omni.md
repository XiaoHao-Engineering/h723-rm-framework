# 四轮全向轮底盘（OMNI4_PLUS）移植说明

从旧工程 `全向轮`（F407 + C 板）移植到本工程（STM32H723 + DM-MC-Board02）的四轮全向轮底盘。
代码只依赖本工程已有的 `dji_motor` 模块和 `PID`/`daemon`，没有引入新的中断、DMA 或任务。

---

## 1. 涉及文件

| 文件 | 作用 |
|---|---|
| `application/rm_example/chassis_omni/chassis_omni.h` | 机械参数、电机 ID/转向、限幅、PID 增益（**要改的参数全在这里**） |
| `application/rm_example/chassis_omni/chassis_omni.c` | 运动学解算 + 电机注册 + 轮速参考下发 |
| `application/rm_example/chassis_omni/chassis_omni.md` | 本文件 |

原工程对应文件：`User_Algorithm/User_Chassis/chassis_kinematics.c`、`User_Application/chassis_config.h`。

---

## 2. 坐标系与轮序

坐标系固定为 **x 向前、y 向左、ω 逆时针为正**，单位 m/s、rad/s。

```
             前 (+x)
               ▲
        [前轮] │
  左 ──────────┼────────── 右
  [左轮]       │        [右轮]
               │ [后轮]
             后 (-x)
```

| 下标 | 宏 | 默认 ID | 正转方向 |
|---|---|---|---|
| 0 | `OMNI_FRONT` | 1 | 把底盘往 **+y**（左）推 |
| 1 | `OMNI_REAR` | 2 | 把底盘往 **-y**（右）推 |
| 2 | `OMNI_LEFT` | 3 | 把底盘往 **-x**（后）推 |
| 3 | `OMNI_RIGHT` | 4 | 把底盘往 **+x**（前）推 |

这样约定后，纯自转（ω>0）时四个轮子**同向正转**，符号最直观。

### 运动学

```
a = 轴距/2，b = 轮距/2
v_front = +vy + a·ω
v_rear  = -vy + a·ω
v_left  = -vx + b·ω
v_right = +vx + b·ω
```

正解：

```
vx    = (v_right - v_left) / 2
vy    = (v_front - v_rear) / 2
ω     = [ (v_front+v_rear)/(2a) + (v_left+v_right)/(2b) ] / 2
```

正解里四个轮子都参与 ω 解算再取平均，抗单轮打滑。

---

## 3. 接入步骤

### 3.1 打开开关

`application/robot_def.h`：

```c
#define DEMO_OMNI_CHASSIS_ENABLE 1
```

### 3.2 初始化

在 `BoardDemoInit()` 里（已接好，打开开关即生效）：

```c
#if DEMO_OMNI_CHASSIS_ENABLE
    ChassisOmniInit();
#endif
```

`ChassisOmniInit()` 会在 `BOARD_CAN1` 上注册 4 个 M3508（ID 1~4），并自动注册 daemon 掉线保护。

### 3.3 周期调用

在 `BoardDemoTask()` 里（100Hz，已接好）：

```c
#if DEMO_OMNI_CHASSIS_ENABLE
    ChassisOmniTask();
#endif
```

**速度内环不需要自己跑**：`DJIMotorControl()` 已经在 `MOTORTASK` 里以 1kHz 运行，`ChassisOmniTask()` 只负责把轮速参考值交给电机模块。

### 3.4 下发速度

```c
ChassisOmniSetVelocity(0.5f, 0.0f, 0.0f);   /* 前进 0.5 m/s */
ChassisOmniSetVelocity(0.0f, 0.3f, 0.0f);   /* 向左平移 */
ChassisOmniSetVelocity(0.0f, 0.0f, 2.0f);   /* 原地逆时针转 2 rad/s */
ChassisOmniStop();                          /* 立即停 */
```

单位是 **底盘速度**，不是轮速；内部会自动做逆解、限幅、归一化。

---

## 4. 参数怎么改

全部在 `chassis_omni.h`：

| 宏 | 说明 |
|---|---|
| `OMNI_WHEELBASE` / `OMNI_TRACK` | **两个轮子中心之间的距离**（不是半径），单位 m |
| `OMNI_WHEEL_R` | 轮半径，76mm 全向轮填 `0.076f` |
| `OMNI_RATIO` | 底盘减速比，**当前 `268/17 ≈ 15.76`**（旧 M3508 P19 为 `3591/187 ≈ 19.2`） |
| `OMNI_ID_*` | 电调 ID，必须 1~4 才能共用 0x200 控制帧 |
| `OMNI_REV_*` | 装反了就改成 `-1`（会同时翻转指令和反馈符号） |
| `OMNI_SPEED_KP/KI/KD` | 轮速环，默认沿用旧工程 `15 / 0.15 / 0` |
| `OMNI_MAX_VX/VY/W` | 底盘速度上限，超过按比例缩放 |
| `OMNI_POWER_MAX` | 四轮总功率上限（按 24V 估算），0 = 关闭 |

### 单位换算（重要）

- `ChassisOmniSetVelocity()` 接收 **m/s**、**rad/s**；
- 内部换算：`轮线速度 → 转子 rpm → 电机模块`；
- `DJIMotorSetRef()` 的参考值和 `measure.speed_aps` 都是 **转子 deg/s**，两者单位一致，PID 才能收敛。

---

## 5. 上机调试顺序（务必按这个来）

1. **先架空**：把车架起来让四个轮子悬空；
2. 只接 **CAN1**、给电调上电，看日志有没有 `[omni] 4-wheel chassis init done`；
3. 单轮验证：临时调用 `ChassisOmniSetVelocity(0, 0, 0)` 确认电机不转；
4. 逐个确认 **ID 与轮子的对应关系**（1=前 2=后 3=左 4=右），不对就改 `OMNI_ID_*`；
5. 逐个确认 **转向**：下发 `ChassisOmniSetVelocity(0.3f, 0, 0)`（前进），
   正确时左右两个轮子应该带动底盘向前；若某个轮子反转，把那一路的 `OMNI_REV_*` 改成 `-1`；
6. 前进/平移/自转各试一次，都对了再落地。

### 常见问题

| 现象 | 原因 |
|---|---|
| 某个轮子飞车 | 反馈方向不对：`OMNI_REV_*` 改 `-1` |
| 前进变成斜着走 | 轮序（前后左右）与 `OMNI_ID_*` 对不上 |
| 自转时某轮反转 | 该轮的 `OMNI_REV_*` 与其余三个不一致 |
| 电机不动但日志正常 | 电调未上电 / CAN 未接 / daemon 判定掉线（看 `[daemon]` 日志） |
| 走起来有啸叫、抖动 | `OMNI_SPEED_KP` 偏大，先降到 8~10 试 |

---

## 6. 遥控（可选）

默认 `OMNI_RC_ENABLE 0`。接好 DR16 后改成 1，通道映射：

| 宏 | 默认 | 含义 |
|---|---|---|
| `OMNI_RC_VX_CH` | 3 | 左摇杆竖直 → 前进/后退 |
| `OMNI_RC_VY_CH` | 2 | 左摇杆水平 → 左右平移 |
| `OMNI_RC_W_CH` | 0 | 右摇杆水平 → 原地旋转 |
| `OMNI_RC_DEAD` | 10 | 摇杆死区 |

`rc_ctrl` 由 `RemoteControlInit()` 创建，需要先在应用里初始化遥控器。

---

## 7. 与旧工程的差异

| 旧工程 | 本工程 | 原因 |
|---|---|---|
| 自己写 `user_dji_motor.c` | 复用 `modules/motor/DJImotor` | 框架已有，避免重复 |
| 自己的 PID 类 | 复用 `PIDInstance`（电机内置三环） | 同上 |
| `SysTick` 任务表调度 1kHz | `MOTORTASK` 1kHz + `ROBOTTASK` 100Hz | 框架任务划分 |
| 自带 serialplot 在线调参 | 未移植 | 需要时用 `vofa` 通道输出 |
| 功率模型按电容/裁判系统 | 简化为 24V × 电流估算 | 没有裁判系统 |
| 保留麦轮 / 45° X 分支 | 只保留十字 OMNI4_PLUS | 本车只用一种布局 |

---

## 8. 移植时踩到的两个坑（已在框架里修掉）

### 8.1 复用电机 config 导致 ID 冲突 + 死锁

`DJIMotorInit()` 会把算出的 `rx_id` **写回你传入的 config 结构体**。如果像下面这样复用同一个结构体：

```c
Motor_Init_Config_s cfg = { ... };
for (i = 0; i < 4; ++i) { cfg.can_init_config.tx_id = id[i]; DJIMotorInit(&cfg); }   /* 错 */
```

第二个电机会带着上一个的 `rx_id` 去注册，命中模块里的 ID 冲突保护：

```c
while (1) LOGERROR("[dji_motor] ID crash ...");
```

而 `BSPInit()` 跑在 `RobotInit()` 的 `__disable_irq()` 区域里，`LOGERROR` 走串口时
`HAL_UART_Transmit` 的超时依赖 `HAL_GetTick`，tick 来自被关掉的 TIM2 中断 → **永久卡死**。

正确写法（本模块已这样写）：

```c
for (i = 0; i < 4; ++i)
{
    cfg.can_init_config.rx_id = 0;      /* 每轮清零 */
    cfg.can_init_config.tx_id = id[i];
    DJIMotorInit(&cfg);
}
```

### 8.2 关中断区域里不要写串口

`bsp/log/bsp_log.c` 已加保护：当 `PRIMASK` 处于关中断状态时**跳过串口输出，只走 RTT**。
原因同上：`HAL_UART_Transmit` 的超时靠 tick，tick 靠中断，关中断时任何"串口卡住"都会变成死锁。

---

## 9. 中断风暴自检

工程里有一个静态检查脚本，专门抓"NVIC 已使能但处理函数是空的"这类致命问题：

```powershell
python tools\irq_audit.py        # 有问题返回 1
python tools\irq_audit.py -v     # 列出所有已使能中断及其来源
```

原理：扫出所有 `HAL_NVIC_EnableIRQ()`，再检查 `stm32h7xx_it.c` 里对应处理函数是否调用了
`HAL_xxx_IRQHandler`。空的处理函数 = 中断标志永不清除 = 中断风暴 = 程序"卡死但无 HardFault"。

**每次用 CubeMX 重新生成代码后建议跑一次。**

---

## 10. 本次移植的验证记录

| 项目 | 结果 |
|---|---|
| 编译（开关开/关两种） | 0 warning / 0 error |
| `omni_motor[0..3]` | 4 个指针全部有效（4 个电机都注册成功） |
| `uwTick` 12 秒连续采样 | `2709 → 5721 → 8729 → 11740`（1kHz 稳定） |
| CFSR / HFSR（4 次采样） | 全部 `0`，无任何异常 |
| CPU 采样 | 空闲任务 / `BMI088_Read` / `HAL_DMA_IRQHandler`（DMA 中断正常服务） |
| `tools/irq_audit.py` | 30 个使能中断全部通过；故意破坏一处能被抓到 |
