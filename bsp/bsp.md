BSP

这是BSP层的说明文档。

> TODO:
> 1. 增加硬件和软件CRC支持,统一不同crc顺序和计算码的调用方式

bsp的功能是提供对片上（MCU中）外设的封装，即单片机芯片内部拥有的功能的封装。**在开发板pcb上集成的模块(蜂鸣器，加热电阻，wifi等)应该放在module层而不是这里**。

bsp应该提供几种接口。包括初始化接口，一般命名为`XXXRegister()`(对于只有一个instance的可以叫`XXXInit()`,但建议统一风格都叫register)；调用此模块实现的必要功能，如通信型外设（CubeMX下的connectivity）提供接收和发送的接口，以及接收完成、发送完成（若有必要，或有发送队列需求）的数据回调函数。

- bsp_tools.h中提供了将bsp数据接收回调函数设置为任务的接口，通过这种方式，可以进一步提高整个系统的实时性，同时保证高优先级的任务一定按时执行。

---

## 本工程(H7 / DM-MC-Board02)的 bsp 文件说明

| 文件 | 功能 | 备注(F4 → H7 的变化) |
| ---- | ---- | ---- |
| `bsp_board.c/h` | 板卡定义、板载外设、电源/按键、DMA 安全段 | **新增**, 换板子只改这里 |
| `bsp_init.c/h` | `BSPInit()` 统一初始化 | DWT(275MHz)/RTT/板载 PWM/WS2812/按键/ADC |
| `bsp_tools.c/h` | 回调 → RTOS 任务 | 不变 |
| `can/` | FDCAN1/2/3 实例、收发、按 id 分发 | bxCAN → FDCAN, 支持 CANFD, 位时序自动计算 |
| `usart/` | 串口 DMA + 空闲中断 | 实例数 3→6, 实例放 DMA 安全区 |
| `spi/` | SPI 主从收发 + 片选 | 仅实例上限修正 |
| `iic/` | I2C 收发/寄存器访问 | 改用 H7 的 `HAL_I2C_Master_Seq_*` |
| `pwm/` | 定时器 PWM | 定时器时钟与周期计算按 H7 重写 |
| `adc/` | 电池电压 + 芯片温度 | **新增实现**(F4 版本是空文件) |
| `gpio/` | GPIO/EXTI 实例 | 不变 |
| `dwt/` | 精确计时/延时 | `DWT_Init(275)` |
| `flash/` | 内部 Flash 读写 | 按 H7 的 128KB 扇区 + 32 字节 flash word 重写 |
| `log/` | SEGGER RTT 日志 | 不变 |
| `usb/` | USB 虚拟串口 | `CDC_Transmit_HS` + 自定义接收回调 |
| `ws2812/` | 板载彩灯(SPI6) | **新增** |

### 两条必须记住的规则

1. **要交给 DMA 的缓冲区, 必须加 `DMA_BUFFER` 宏**
   (定义在 `bsp_board.h`, 放到 AXI SRAM 的 `.dma_buffer` 段)。
   DTCM(0x20000000)不能被 DMA 访问, 而 `malloc` 默认从 DTCM 分配。

2. **板载外设的引脚/句柄统一写在 `bsp_board.h`**
   module 层不要直接写 `&hfdcan1`、`GPIO_PIN_x`, 用 `BOARD_CAN1`、`BOARD_PWM_BUZZER` 这样的宏,
   以后换板子时只改 bsp 层。