# CubeMX 重新生成代码后的检查清单

本工程的 RTOS、部分中断转发是**手工接的**，`RM.ioc` 里 `FREERTOS = Disable`。
用 CubeMX 点 **GENERATE CODE** 之后，必须按下面清单核对一遍，否则会出现
“编译通过、烧录成功、但什么都不动”的静默故障。

---

## 1. 一定会被覆盖、必须检查的文件

| 文件 | 会被覆盖什么 | 检查什么 |
|---|---|---|
| `Core/Src/main.c` | 外设初始化段 | 关键代码是否还在 `USER CODE` 区内（见下表） |
| `Core/Src/stm32h7xx_it.c` | 所有中断处理函数 | ① `TIM2_IRQHandler` 里有 `HAL_TIM_IRQHandler(&htim2)` ② 16 个 DMA 流有 `HAL_DMA_IRQHandler(&hdma_xxx)` ③ `SPI2_IRQHandler` 有 `HAL_SPI_IRQHandler(&hspi2)` |
| `Core/Src/stm32h7xx_hal_timebase_tim.c` | HAL 时基 | `HAL_InitTick` 用的是 TIM2，且 `.ioc` 里 `NVIC.TimeBase=TIM2_IRQn` |
| `Core/Inc/FreeRTOSConfig.h` | **如果将来在 .ioc 里打开 FREERTOS** | heap 32KB、`configSUPPORT_STATIC_ALLOCATION 1`、7 个优先级、TICK 1000 |
| `Core/Src/gpio.c` | 引脚初始电平 | `PC15`(POWER_5V) 必须是 **高电平**（5V 打开） |

## 2. 必须在 USER CODE 区里的关键代码

| 位置 | 内容 | 为什么 |
|---|---|---|
| `main.c` → `USER CODE BEGIN 2` | `RobotInit();` + `osKernelStart();` | 放在生成区会被删 → 调度器不启动 → 任务全不跑 |
| `main.c` → `USER CODE BEGIN 2` | `MinimalTestInit()`（自检固件） | 同理 |
| `stm32h7xx_it.c` → `TIM2_IRQn 1` | `HAL_TIM_IRQHandler(&htim2);` | 缺了它 → uwTick 不涨 → 所有 HAL_Delay 死等 |
| `stm32h7xx_it.c` → 各 DMA `IRQn 1` | `HAL_DMA_IRQHandler(&hdma_xxx);` | 缺了它 → 中断风暴 |
| `stm32h7xx_it.c` → `SPI2_IRQn 1` | `HAL_SPI_IRQHandler(&hspi2);` | 同上 |

## 3. 重新生成后最容易踩的三个坑

### 3.1 `osKernelStart()` 被删
现象：灯不亮、蜂鸣器不响、串口只有 `[boot] ...` 之后就没动静。
判断：调试器暂停，PC 停在 `main` 的 `while(1)` 里空转。
处理：把 `osKernelStart()` 放回 `USER CODE BEGIN 2`。

### 3.2 `SVC_Handler / PendSV_Handler / SysTick_Handler` 重复定义
CubeMX 在“没有 RTOS”的项目里会生成这三个函数，而 FreeRTOS 的 `port.c`
通过 `FreeRTOSConfig.h` 里的宏也定义了它们 →

```
multiple definition of `SVC_Handler'
```

处理：删掉 `stm32h7xx_it.c` 里这三个空函数（保留 `FreeRTOSConfig.h` 的宏映射）。

### 3.3 TIM2 / DMA 中断又变回空函数
现象：`uwTick` 恒为 0、`HAL_Delay` 永不返回、或一上电就中断风暴。
处理：跑一遍 `python tools/irq_audit.py`，它会直接指出哪个中断缺转发。

## 4. 生成后建议的执行顺序

```powershell
# 1) 静态检查中断(不通过就别烧)
python tools/irq_audit.py

# 2) 编译
cmake --preset Debug
cmake --build --preset Debug

# 3) 确认关键符号还在
arm-none-eabi-nm build/Debug/RM.elf | findstr "osKernelStart xTaskCreate HAL_TIM_IRQHandler"

# 4) 烧录 + 上电看现象(灯 -> 滴 -> 开机音乐 -> 绿灯心跳)
```

## 5. 想彻底避免这些坑

两条路：

- **A（推荐，现状）**：保持 FreeRTOS 手工接入，改外设时只改 `.ioc`，
  生成后按本清单核对。优点是 `FreeRTOSConfig.h` 完全由我们控制。
- **B**：在 `.ioc` 里打开 `FREERTOS`（CMSIS_V1、TICK_RATE 1000、MAX_PRIORITIES 7、
  TOTAL_HEAP 32KB、时基 TIM2、**不要**生成默认任务），让 CubeMX 接管。
  风险：它会**覆盖** `Core/Inc/FreeRTOSConfig.h`，生成前先备份。

---

## 6. FreeRTOS 相关（v2 迁移后新增的必查项）

| 检查点 | 位置 | 说明 |
|---|---|---|
| **生成区的重复启动块** | `Core/Src/main.c` 的 `/* USER CODE END 2 */` 之后 | CubeMX 会再插一段 `osKernelInitialize(); MX_FREERTOS_Init(); osKernelStart();`。本工程已在 USER CODE 2 里启动过，必须把这段删掉（保留注释即可） |
| **栈溢出/堆耗尽 hook** | `Core/Src/freertos.c` 的 `USER CODE 4` / `USER CODE 5` | 带日志的实现已放在这里（CubeMX 保留 USER CODE 区）。**不要再在 `robot_task.c` 里定义同名函数**，否则 `multiple definition of vApplicationStackOverflowHook` |
| `configASSERT` | `Core/Inc/FreeRTOSConfig.h` 的 `USER CODE 1` | 我们的"先打印再停机"版本在这里，重新生成会保留 |
| 静态内存 hook | `application/robot_task.c` | `vApplicationGetIdleTaskMemory` / `vApplicationGetTimerTaskMemory`，`configUSE_TIMERS=1` 时二者都需要 |
| 任务栈单位 | `application/robot_task.c` | v2 的 `osThreadAttr_t.stack_size` 是**字节**：1024 字 → 4096 字节 |

生成后快速自检：

```powershell
python tools\irq_audit.py     # 中断转发
python tools\ioc_audit.py     # .ioc 与 FreeRTOSConfig.h 一致性
cmake --build build/Debug      # 必须 0 warning / 0 error
```
