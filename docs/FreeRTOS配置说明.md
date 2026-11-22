# FreeRTOS 配置说明（CubeMX 参数 ↔ 本工程实际值）

本工程的 FreeRTOS 是**手工接入**的（`.ioc` 里 FREERTOS = Disable），
配置文件是 `Core/Inc/FreeRTOSConfig.h`。本文做两件事：

1. 解释 CubeMX 里那些参数分别在管什么、为什么这么取值；
2. 给出"如果要让 CubeMX 接管，应该怎么填"的对照表（以及会有什么代价）。

---

## 1. 第一件事：Interface 选哪个

| 选项 | 说明 | 能不能选 |
|---|---|---|
| `Disable` | CubeMX 不生成任何 RTOS 代码 | ✅ **当前状态**，最稳 |
| `CMSIS_V1` | 老接口：`osThreadDef / osThreadCreate / osDelay / osSignalSet` | ⚠️ 新版 CubeMX 已禁用（灰掉），只能用 V2 |
| `CMSIS_V2` | 新接口：`osThreadNew` + `osThreadAttr_t` | ✅ 新版 CubeMX 唯一可选；框架需迁移约 12 行 |

`robot_task.c` 里现在是：

```c
osThreadDef(instask, StartINSTASK, osPriorityAboveNormal, 0, 1024);
osThreadCreate(osThread(instask), NULL);
```

这是 **CMSIS-RTOS v1** 的写法。所以要么保持 `Disable`，要么选 `CMSIS_V1`，**不能选 V2**。

---

## 2. 参数对照表（CubeMX 默认值 vs 本工程实际值）

| CubeMX 参数 | CubeMX 默认 | 本工程 | 是否要一致 | 作用 / 讲究 |
|---|---|---|---|---|
| `ENABLE_MPU` | Disabled | `configENABLE_MPU 0` | 一致 | RTOS 的 MPU 任务隔离是另一套东西；我们用 MPU 只是为了配 cache 属性，不要开 |
| `ENABLE_FPU` | Disabled | `configENABLE_FPU 1` | **必须开** | 任务里全是 float（EKF、矩阵、PID）。真正保存 FPU 上下文的是 port 的 PendSV（lazy stacking），但这个开关保证生成一致 |
| `USE_PREEMPTION` | Enabled | `1` | 一致 | 抢占式调度。关掉就变成"等让出才切换"，1kHz 控制环会失步 |
| `CPU_CLOCK_HZ` | SystemCoreClock | 同 | 一致 | 275MHz，改主频自动跟随；填错 tick 频率就错，`osDelay(1)` 不再是 1ms |
| `TICK_RATE_HZ` | 1000 | `1000` | 一致 | 1ms 分辨率。再高没意义（HAL 时基也是 1kHz），只会增加中断开销 |
| `MAX_PRIORITIES` | 56 | **7** | 建议 7 | CMSIS v1 的 `osPriority` 只有 7 档（Idle→Realtime）；每个优先级占一个 ready list，56 是浪费 RAM |
| `MINIMAL_STACK_SIZE` | 128 Words | `128` | 一致 | =512B，只给 **idle/timer** 任务用；我们自己的任务在 `osThreadDef` 里单独给（256~1024 words） |
| `MAX_TASK_NAME_LEN` | 16 | **32** | 建议 32 | 任务名被截断不会报错，但调试时看不出是谁 |
| `USE_16_BIT_TICKS` | Disabled | `0` | 一致 | 32 位 tick，49 天不溢出 |
| `IDLE_SHOULD_YIELD` | Enabled | 默认 1 | 一致 | 空闲任务和同优先级任务让出 CPU |
| `USE_MUTEXES` | Enabled | `1` | 一致 | CMSIS v1 的 `osMutex` 需要它 |
| `USE_RECURSIVE_MUTEXES` | Enabled | `0` | 可关 | 省一点代码；用到递归锁再开 |
| `USE_COUNTING_SEMAPHORES` | Enabled | `1` | 一致 | 计数信号量 |
| `QUEUE_REGISTRY_SIZE` | 8 | `8` | 一致 | 给队列起名字，调试用 |
| `USE_APPLICATION_TASK_TAG` | Disabled | 未定义 | 一致 | 不用 |
| `ENABLE_BACKWARD_COMPATIBILITY` | Enabled | — | 仅 V2 相关 | 选 V1 时无意义 |
| `USE_PORT_OPTIMISED_TASK_SELECTION` | Disabled | **1** | 建议 1 | 用 `CLZ` 指令找最高优先级，快；要求 `MAX_PRIORITIES ≤ 32`（我们 7，满足） |
| `USE_TICKLESS_IDLE` | Disabled | 未定义 | **必须关** | 见下面第 3.2 节，开了会让 `HAL_GetTick()` 不再稳定增长 |
| `USE_TASK_NOTIFICATIONS` | Enabled | `1` | 一致 | 任务间轻量通知 |
| `RECORD_STACK_HIGH_ADDRESS` | Disabled | 未定义 | 可开 | 配合栈溢出检查能报出更准的位置，多 4 字节/任务 |

截图里看不到、但必须自己核对的：

| 参数 | 本工程值 | 说明 |
|---|---|---|
| `configTOTAL_HEAP_SIZE` | **32 KB** | CubeMX 默认只有 15 KB。任务是动态创建的（`osThreadCreate`），每加一个任务/队列都吃堆 |
| `configSUPPORT_STATIC_ALLOCATION` | **1** | 让 idle 任务用我们提供的静态内存（`robot_task.c` 里的 `vApplicationGetIdleTaskMemory`），省堆 |
| `configCHECK_FOR_STACK_OVERFLOW` | **2** | 每次切换检查栈指针，越界立刻调 hook 打印，而不是"跑飞了在别处莫名死掉" |
| `configUSE_MALLOC_FAILED_HOOK` | **1** | `pvPortMalloc` 失败时回调，能在串口看到"堆耗尽了" |
| `configMAX_SYSCALL_INTERRUPT_PRIORITY` | **5** | 见第 3.1 节，这是最容易出错的一条 |
| `configASSERT` | 自定义 | 断言失败先串口打印再停机，而不是静默 `for(;;)` |

---

## 3. 三条铁律（原理）

### 3.1 中断优先级：数值必须 ≥ 5

Cortex-M 的中断优先级**数值越小优先级越高**。FreeRTOS 在临界区把 `BASEPRI` 设成
`configMAX_SYSCALL_INTERRUPT_PRIORITY = 5`，意思是：

- **优先级 5~15 的中断**：会被临界区屏蔽，可以安全调用 `FromISR` 版 API；
- **优先级 0~4 的中断**：不受屏蔽，**绝对不能**调用任何 FreeRTOS API
  （会破坏内核链表，表现为随机 HardFault 或任务再也不切换）。

本工程现状（都是合规的）：

```
所有外设中断(DMA/UART/FDCAN/SPI2/BDMA) = 5
PendSV                                  = 15   ← 必须最低
HAL 时基 TIM2                           = 15
```

改 CubeMX 的 NVIC 页面时，**不要把任何中断调成 0~4**。

### 3.2 tick 与 HAL 时基必须分开

```
SysTick  -> 只给 FreeRTOS (xPortSysTickHandler)
TIM2     -> 只给 HAL_GetTick() (NVIC.TimeBase=TIM2_IRQn)
```

- 两者都用 SysTick 会互相打架（CubeMX 强制你用 TIM 做 HAL 时基，就是这个原因）；
- TIM2 优先级是 15，会被 FreeRTOS 临界区延迟 → `HAL_GetTick()` 在长临界区里会
  **丢 tick**。所以：**不要用 `HAL_GetTick()` 做微秒级计时**，精确计时用 DWT；
- `configUSE_TICKLESS_IDLE` 打开后空闲时 tick 会停，`HAL_GetTick()` 直接不涨，
  所有 `HAL_Delay`/超时会失效 → 本工程必须保持 **Disabled**。

### 3.3 关中断区域里不要调用依赖 tick 的东西

`RobotInit()` 一开始就 `__disable_irq()`，`BSPInit()` 整段都在关中断状态。
此时 TIM2 中断不会触发 → `HAL_GetTick()` 冻结 →
`HAL_Delay()`、`HAL_UART_Transmit()`（带超时）都可能**永久卡死**。

已经做的保护：
- `BSPLogOutput()` 检测到关中断就只走 RTT，不写串口；
- `BoardBeep()` 在关中断区里用 DWT 计时而不是 `HAL_GetTick()`。

---

## 4. 如果真要让 CubeMX 接管（代价清单）

选 `CMSIS_V1` 之后点 GENERATE CODE，会发生：

| 现象 | 后果 |
|---|---|
| 覆盖 `Core/Inc/FreeRTOSConfig.h` | 丢 32KB 堆、栈检查、malloc hook、自定义 assert、7 优先级 |
| 生成 `MX_FREERTOS_Init()` + 一个**默认任务** | 和 `RobotInit()` 的任务表重复 |
| 在 `main.c` 生成区插入 `osKernelStart()` | 和我们 `USER CODE 2` 里的重复 → 编译错或双启动 |
| **把 port 从 `ARM_CM4F` 换成 `ARM_CM7/r0p1`** | `CMakeLists.txt` 里写死了 `ARM_CM4F` → 直接编不过 |
| 生成 `cmsis_os.c/h` 到 Middlewares | 和现有文件重复定义 |

**结论：保持 `Disable`（现状）最稳。** 若确实要接管，顺序是：

1. 备份 `Core/Inc/FreeRTOSConfig.h`；
2. 在 CubeMX 里选 `CMSIS_V1`，参数按第 2 章的表填；
3. 生成后立刻 diff 这 5 个文件：`FreeRTOSConfig.h` / `main.c` / `stm32h7xx_it.c` / `CMakeLists.txt` / `Middlewares/Third_Party/FreeRTOS/Source/portable/`；
4. 跑 `python tools/irq_audit.py`；
5. 编译后确认符号：`osKernelStart`、`xTaskCreate` 在，且没有 `MX_FREERTOS_Init`。
