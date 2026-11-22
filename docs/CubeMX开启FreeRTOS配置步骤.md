# 在 CubeMX 里开启 FreeRTOS（CMSIS_V1）配置步骤

目标：让 CubeMX 生成的 RTOS 配置**与当前框架代码等价**，生成后只需少量收尾修改。

> 生成前已备份到 `docs/backup_pre_cubemx_rtos/`：
> `FreeRTOSConfig.h` / `main.c` / `stm32h7xx_it.c` / `CMakeLists.txt` / `stm32cubemx_CMakeLists.txt` / `RM.ioc`
> 生成后若有问题，直接从这里拷回去即可回滚。

---

## 步骤 1：选接口版本

`Pinout & Configuration` → `Middleware and Software Packs` → `FREERTOS`
→ **Interface = `CMSIS_V2`**

> ⚠️ 新版 CubeMX 已把 `CMSIS_V1` 标为 deprecated（灰色不可选），只能生成 V2。
> 现有框架代码用的是 V1 接口，生成后需要**小幅迁移**（约 12 行，见文末第 9 节），
> 迁移后行为等价。`osDelay()` / `osKernelStart()` 在 V2 里同名同义，不用改。

---

## 步骤 2：Config parameters（逐个填）

| 参数 | 填这个值 | 依据 |
|---|---|---|
| `USE_PREEMPTION` | **Enabled** | 1kHz 控制环需要抢占 |
| `USE_IDLE_HOOK` | Disabled | 不用 |
| `USE_TICK_HOOK` | Disabled | 不用 |
| `TICK_RATE_HZ` | **1000** | 1ms 分辨率 |
| `MAX_PRIORITIES` | **56** | V2 的 `osPriority` 有 56 档；留 56 最稳（只多占约 224B RAM） |
| `MINIMAL_STACK_SIZE` | **128** (Words) | 512B，只给 idle/timer 任务 |
| `MAX_TASK_NAME_LEN` | **32** | 任务名带长前缀，16 会被截断 |
| `USE_MUTEXES` | Enabled | `osMutex` 需要 |
| `USE_RECURSIVE_MUTEXES` | Disabled | 当前没用到 |
| `USE_COUNTING_SEMAPHORES` | Enabled | 需要 |
| `QUEUE_REGISTRY_SIZE` | 8 | 队列命名（调试用） |
| `USE_PORT_OPTIMISED_TASK_SELECTION` | **Enabled** | 用 CLZ 找最高优先级；`MAX_PRIORITIES ≤ 32` 满足 |
| `USE_TICKLESS_IDLE` | **Disabled** | ⚠️ 开了 `HAL_GetTick()` 会停涨，全工程超时失效 |
| `USE_TASK_NOTIFICATIONS` | Enabled | 需要 |
| `ENABLE_MPU` | Disabled | RTOS 的 MPU 隔离不用 |
| `ENABLE_FPU` | **Enabled** | 任务里全是 float（EKF/矩阵/PID） |
| `USE_16_BIT_TICKS` | Disabled | 32 位 tick |
| `IDLE_SHOULD_YIELD` | Enabled | 默认行为 |
| `TOTAL_HEAP_SIZE` | **32768** | 任务/队列/电机实例都从堆里来；CubeMX 默认 15KB 不够 |
| `SUPPORT_STATIC_ALLOCATION` | **Enabled** | 让 idle 任务用静态内存（省堆） |
| `SUPPORT_DYNAMIC_ALLOCATION` | Enabled | `osThreadCreate` 是动态的 |
| `MEMORY_MANAGEMENT` | **heap_4**（可选时） | 与现有 `heap_4.c` 一致 |

---

## 步骤 3：Advanced settings

| 参数 | 填这个值 | 原因 |
|---|---|---|
| `CHECK_FOR_STACK_OVERFLOW` | **Option 2** | 栈越界立刻报警，而不是"跑飞了在别处莫名死" |
| `USE_MALLOC_FAILED_HOOK` | **Enabled** | 堆耗尽时能打印出来 |
| `RECORD_STACK_HIGH_ADDRESS` | Enabled（可选） | 溢出错时能报更准的位置 |
| `USE_APPLICATION_TASK_TAG` | Disabled | 不用 |
| `ENABLE_BACKWARD_COMPATIBILITY` | Enabled（V2 才有效，V1 忽略） | — |

---

## 步骤 4：删掉默认任务（重要）

`Tasks and Queues` 里 CubeMX 会自动建一个 `defaultTask`。
**右键 → Remove**，把它删掉。

> 原因：框架的任务（INS/MOTOR/DAEMON/ROBOT/UI）都在 `robot_task.c` 里用
> `osThreadCreate` 创建。留着 `defaultTask` 只会白占一份栈和优先级。

`Timers and Semaphores` 保持空。

---

## 步骤 5：确认时基

`System Core` → `SYS` → **Timebase Source = `TIM2`**

> 必须是 TIM（本工程是 TIM2），不能是 SysTick。
> SysTick 要留给 FreeRTOS（`xPortSysTickHandler`），两者混用会互相打架。

---

## 步骤 6：确认中断优先级

`System Core` → `NVIC`：

- 所有外设中断（DMA/UART/FDCAN/SPI2/BDMA）**保持 5**；
- **不要**把任何中断改成 0~4。

> 原理：FreeRTOS 临界区把 `BASEPRI` 设为 5，优先级 0~4 的中断不会被屏蔽，
> 一旦它们调用 FreeRTOS API 就会破坏内核链表（随机 HardFault）。

---

## 步骤 7：Project Manager

- `Toolchain / IDE` = **CMake**（保持现状，否则生成的 `cmake/stm32cubemx/CMakeLists.txt` 会变）；
- 确认没有勾选会把代码生成到子目录的选项（`Generate Under Root` 之类）。

---

## 步骤 8：GENERATE CODE

生成后**先别编译**，把工程交给我做收尾（见下）。

---

## 生成后会发生什么 / 我要做什么

| 文件 | CubeMX 会怎样 | 收尾动作 |
|---|---|---|
| `Core/Inc/FreeRTOSConfig.h` | **整个覆盖** | 合并回我们的：32KB 堆、自定义 `configASSERT`（打印后停机）、`configMAX_SYSCALL_INTERRUPT_PRIORITY=5`、`RECORD_STACK_HIGH_ADDRESS` |
| `Core/Src/freertos.c` | 新增，含 `MX_FREERTOS_Init()` | 不调用它（我们的任务是 `RobotInit()` 建的），或直接排除 |
| `Core/Src/main.c` | 生成区插入 `osKernelStart()` | 删掉重复的那个，保留 `USER CODE 2` 里的 |
| `Core/Src/stm32h7xx_it.c` | 可能生成 `SVC/PendSV/SysTick_Handler` | 删掉这三个空函数（否则和 port 重复定义） |
| `Middlewares/.../portable/GCC/ARM_CM7/r0p1/` | 新增 port | `CMakeLists.txt` 已改成**自动识别** port，无需手改 |
| `cmake/stm32cubemx/CMakeLists.txt` | 加入 FreeRTOS 源文件与 include | 与手写 CMakeLists 去重（同一文件重复添加要删掉一处） |
| `Core/Src/main.c` 的 `TIM2_IRQHandler` 修复 | 会被重新生成成空函数 | 把 `HAL_TIM_IRQHandler(&htim2)` 补回 USER CODE 区 |
| 16 个 DMA 流 + SPI2 的中断转发 | 同上，会变回空函数 | 全部补回（在 USER CODE 区） |

---

## 生成后我按这个顺序验收

```powershell
# 1) 中断转发检查(必须通过)
python tools\irq_audit.py

# 2) 编译
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug `
      -DUSE_RM_EXAMPLE=OFF -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build/Debug

# 3) 符号检查: 这些必须在, MX_FREERTOS_Init 最好不在
arm-none-eabi-nm build\Debug\RM.elf | findstr "osKernelStart xTaskCreate vTaskStartScheduler MX_FREERTOS_Init"

# 4) 烧录后看现象: 滴一声 -> 开机音乐 -> 绿灯心跳
```

## 一个提醒

CubeMX 生成的是 **ARM_CM7/r0p1** port，而我们原来是 **ARM_CM4F**。
两者功能等价（M7 的 port 对 cache/FPU 处理更细），CMake 已自动识别。
但如果生成后出现 `hard fault` 或调度异常，第一个怀疑对象就是 port 切换，
届时把 `docs/backup_pre_cubemx_rtos/` 里的版本拷回即可回到 ARM_CM4F。

---

## 9. 生成后必须做的代码迁移（V1 → V2）

### 9.1 只涉及 3 个文件

| 文件 | 现在（V1） | 改成（V2） |
|---|---|---|
| `application/robot_task.c` | `osThreadDef(...)` + `osThreadCreate(osThread(x), NULL)`；句柄类型 `osThreadId` | `osThreadNew(func, NULL, &attr)`；句柄类型 `osThreadId_t` |
| `bsp/bsp_tools.c` | `osThreadDef_t` 手工填 + `osThreadCreate` + `osSignalWait` | `osThreadNew` + `osThreadAttr_t` + `osThreadFlagsWait` |
| `Core/Src/main.c` | `osKernelStart()` | 不变 |
| 9 个文件 | `#include "cmsis_os.h"` | 改成 `#include "cmsis_os2.h"` |

`osDelay()`、`osKernelStart()` 在 V2 中同名同义，**不用改**。

### 9.2 ⚠️ 最大的坑：栈大小单位变了

| | V1 `osThreadDef(..., stacksize)` | V2 `attr.stack_size` |
|---|---|---|
| 单位 | **字（word，4 字节）** | **字节（byte）** |

现有 5 个任务的栈必须这样换算，否则栈只有 1/4，跑起来就 HardFault：

| 任务 | V1 写法 | V2 必须写 |
|---|---|---|
| INS | `1024` words | `4096` |
| MOTOR | `512` | `2048` |
| DAEMON | `256` | `1024` |
| ROBOT | `1024` | `4096` |
| UI | `512` | `2048` |
| CallbackTask(bsp_tools) | `128` | `512` |

### 9.3 信号量 API 对应关系

| V1 | V2 |
|---|---|
| `osSignalSet(tid, sig)` | `osThreadFlagsSet(tid, flags)` |
| `osSignalWait(sig, timeout)` | `osThreadFlagsWait(flags, osFlagsWaitAny, timeout)` |
| `osSignalClear(tid, sig)` | `osThreadFlagsClear(flags)` |
| `osWaitForever` | `osWaitForever`（同名，仍在） |

返回值和"信号是否被消费"的语义略有差别：V1 的 `osSignalWait` 会**消费**信号，
V2 的 `osThreadFlagsWait` 默认也会清标志，行为一致。

### 9.4 迁移后的等价性检查

```c
/* V1 */
osThreadDef(motortask, StartMOTORTASK, osPriorityNormal, 0, 512);
osThreadCreate(osThread(motortask), NULL);

/* V2 —— 注意 stack_size 是字节 */
static const osThreadAttr_t motortask_attr = {
    .name = "motortask",
    .priority = osPriorityNormal,
    .stack_size = 2048,          /* 512 words x 4 */
};
osThreadNew(StartMOTORTASK, NULL, &motortask_attr);
```

完成后必须实测：串口出现 5 条 `[freeRTOS] xxx Task Start`，
且 `uwTick` 稳定增长（说明调度器在跑、栈没爆）。
