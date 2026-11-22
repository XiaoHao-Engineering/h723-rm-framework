/**
 * @file bsp_board.h
 * @brief 板级定义与外设映射 —— 达妙 DM-MC-Board02 (STM32H723VGT6)
 *
 * 这个文件是"换板子"的唯一入口: bsp 层的其他文件都从这里取板载外设句柄,
 * 需要把工程跑在别的板上时, 只需要改这一个文件.
 *
 * 引脚与外设对照表(摘自《DM-MC-Board02 使用说明书 V1.1》与官方例程):
 *   CAN1   PD0/PD1      CAN2  PB5/PB6      CAN3  PD12/PD13
 *   USART1 PA9/PA10     USART2 PD5/PD6(RS485, DE=PD4)   USART3 PD8/PD9(RS485, DE=PB14)
 *   UART7  PE8/PE7      USART10 PE3/PE2   UART5 PD2 (SBUS, 100000 8E2, 板载反相)
 *   SPI2   PB13/PC1/PC2 (BMI088, CS: PC0 加速度计 / PC3 陀螺仪)
 *   SPI1   PE15/PB3/PB4/PD7 (LCD)        SPI6  PA7 (WS2812)
 *   TIM3_CH4 PB1 (IMU 加热电阻)          TIM12_CH2 PB15 (蜂鸣器)
 *   PWM1~4 PE9(TIM1_CH1) PE13(TIM1_CH3) PA0(TIM5_CH1) PA2(TIM5_CH3)
 *   ADC1_INP4 PC4 (电源电压采集)         按键 PA15 / PA5
 *   USB    PA11/PA12 (OTG_HS, FS)
 */

#ifndef BSP_BOARD_H
#define BSP_BOARD_H

#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

#define BOARD_NAME         "DM-MC-Board02"
#define BOARD_MCU_NAME     "STM32H723VGT6"

/**
 * @note 编译期"标称"主频, 仅用于文档/日志参考。
 *       运行时请一律用 HAL_RCC_GetSysClockFreq() / HAL_RCC_GetHCLKFreq() /
 *       HAL_RCC_GetPCLK1Freq() / SystemCoreClock 取值, 这样改了时钟树也不会算错。
 *
 *       当前工程: PLL1 = 24MHz/2 x 68.75 = 825MHz VCO, /3 = 275MHz SYSCLK,
 *                 HCLK=DIV1=275MHz, APB1/2=DIV2=137.5MHz, VOS0。
 *       (H72x/H73x 的 AXI/AHB 上限就是 275MHz; CPU 上限 520MHz, 打开
 *        CPU_FREQ_BOOST 选项字节后 550MHz, 此时 HPRE=/2 使总线仍为 275MHz)
 */
#define BOARD_CPU_FREQ_MHZ 275U /* 标称值, 仅供参考 */

#define ARRAY_LEN(x) (sizeof(x) / sizeof((x)[0]))

/**
 * @brief DMA 安全内存段
 *
 * H7 的 DTCM(0x20000000) **不能** 被 DMA 访问, 而链接脚本默认把 .data/.bss 放在 DTCM.
 * 因此所有 DMA 缓冲区(串口接收缓冲, ADC 缓冲, 以及所有 malloc 出来的实例)都放在
 * RAM_D1(0x24000000) 的 .dma_buffer 段中, 同时 MPU 把该区域配置为非 Cache,
 * 这样 DMA 与 CPU 之间不需要做 cache 维护.
 *
 * 用法: uint8_t buf[64] DMA_BUFFER;
 */
#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned(32)))

/* ============================ CAN / FDCAN ============================ */
#define BOARD_CAN1 (&hfdcan1) /* CAN1: PD0(RX) / PD1(TX) */
#define BOARD_CAN2 (&hfdcan2) /* CAN2: PB5(RX) / PB6(TX) */
#define BOARD_CAN3 (&hfdcan3) /* CAN3: PD12(RX) / PD13(TX) */
#define BOARD_CAN_CNT 3

/* ============================== 串口 ================================ */
#define BOARD_USART1   (&huart1)  /* 对外串口, 带 5V 供电 */
#define BOARD_USART2   (&huart2)  /* RS485-2, 硬件 DE(PD4) */
#define BOARD_USART3   (&huart3)  /* RS485-3, 硬件 DE(PB14) */
#define BOARD_UART5    (&huart5)  /* SBUS: 100000-8E2, 板载反相 */
#define BOARD_UART7    (&huart7)  /* 对外串口 */
#define BOARD_USART10  (&huart10) /* 对外串口 */
#define BOARD_USART_CNT 6

/* =============================== SPI ================================ */
#define BOARD_SPI_IMU (&hspi2) /* BMI088: SCK=PB13 MOSI=PC1 MISO=PC2 */
#define BOARD_IMU_ACCEL_CS_PORT ACCEL_CS_GPIO_Port
#define BOARD_IMU_ACCEL_CS_PIN  ACCEL_CS_Pin
#define BOARD_IMU_GYRO_CS_PORT  GYRO_CS_GPIO_Port
#define BOARD_IMU_GYRO_CS_PIN   GYRO_CS_Pin

/* =============================== ADC ================================ */
#define BOARD_ADC_VBAT    (&hadc1) /* PC4 / ADC1_INP4: 电源电压采集 */
#define BOARD_ADC_TEMP    (&hadc3) /* 芯片内部温度传感器 */
#define BOARD_VBAT_R1_KOHM 100.0f  /* 分压上桥 100k, 需要按实际电路修改 */
#define BOARD_VBAT_R2_KOHM 10.0f   /* 分压下桥 10k  */
#define BOARD_VBAT_ADV_REF 3.3f    /* ADC 参考电压 */

/* ============================== 定时器 ============================== */
#define BOARD_TIM_IMU_HEATER (&htim3) /* 由 CubeMX 初始化的 TIM3 */
#define BOARD_IMU_HEATER_CH TIM_CHANNEL_4

/* ============================== 日志输出 ============================ */
/**
 * @brief 日志的串口输出通道
 *
 * RTT 日志需要 J-Link 才能看; 如果手头只有 ST-Link / DAPLink,
 * 可以把 USART1(PA9=TX, PA10=RX, 115200-8N1)接一个 USB-TTL 到电脑,
 * 日志会同时从这里输出(纯文本, 无颜色).
 *
 * 不需要串口日志时, 注释掉下面这一行即可.
 */
#define BSP_LOG_UART BOARD_USART1
#define BSP_LOG_UART_BAUDRATE 115200

/* ============================== USB ================================= */
/**
 * @brief 是否初始化 USB(虚拟串口)
 *
 * 用不到 USB(例如跑电机实验、排查"中断风暴/卡顿")时把它设成 0:
 *   - 少一个可能反复进出 OTG_HS 中断的外设;
 *   - 少一个 48MHz 时钟域;
 *   - 视觉/上位机通信需要 USB CDC 时再改回 1.
 */
#define BSP_USB_ENABLE 1

/* CubeMX 没有配置、由 bsp 自行初始化的定时器(见 BoardPWMHardwareInit) */
extern TIM_HandleTypeDef bsp_htim1;  /* PE9/PE13  → 板载 PWM1/PWM2 */
extern TIM_HandleTypeDef bsp_htim5;  /* PA0/PA2   → 板载 PWM3/PWM4 */
extern TIM_HandleTypeDef bsp_htim12; /* PB15      → 蜂鸣器 */

/**
 * @brief 板载 PWM 通道编号, 与丝印/说明书一致
 */
typedef enum
{
    BOARD_PWM_1 = 0,      /* PE9  TIM1_CH1 */
    BOARD_PWM_2,          /* PE13 TIM1_CH3 */
    BOARD_PWM_3,          /* PA0  TIM5_CH1 */
    BOARD_PWM_4,          /* PA2  TIM5_CH3 */
    BOARD_PWM_BUZZER,     /* PB15 TIM12_CH2, 板载无源蜂鸣器 */
    BOARD_PWM_IMU_HEATER, /* PB1  TIM3_CH4,  BMI088 加热电阻 */
    BOARD_PWM_CNT
} BoardPWMChannel_e;

/**
 * @brief 取得板载 PWM 通道对应的定时器句柄与通道号
 * @note 用于 PWMRegister(&(PWM_Init_Config_s){.htim=..., .channel=...})
 */
TIM_HandleTypeDef *BoardPWMGetHandle(BoardPWMChannel_e ch, uint32_t *channel_out);

/**
 * @brief 初始化 CubeMX 未配置的定时器(TIM1/TIM5/TIM12)及其 GPIO
 * @note 由 BSPInit() 调用
 */
void BoardPWMHardwareInit(void);

/**
 * @brief 让板载蜂鸣器直接响一声(阻塞, 单位 ms)
 *
 * @note 给"上电自检"用: 不经过 buzzer module, 也不依赖 RTOS,
 *       所以它响不响可以直接判断"5V + 定时器 + 蜂鸣器"这一路是否正常.
 *
 * @param freq_hz     发声频率(Hz), 2~4kHz 最接近这颗无源蜂鸣器的谐振点
 * @param duration_ms 持续时间(ms)
 */
void BoardBeep(uint32_t freq_hz, uint32_t duration_ms);

/**
 * @brief 开始/停止蜂鸣器发声(非阻塞, 用于需要自己控制时长/节奏的场合)
 */
void BoardBeepStart(uint32_t freq_hz);
void BoardBeepStop(void);

/* ============================ 电源控制 ============================== */
/**
 * @brief 5V 对外供电使能(PC15, 给 PWM 舵机 / CAN 收发器 / 串口模块供电)
 */
void BoardPower5V(uint8_t enable);

/**
 * @brief 可控电源输出使能
 * @param ch 1 => OUT_EN1(PC14), 2 => OUT_EN2(PC13)
 */
void BoardPowerOut(uint8_t ch, uint8_t enable);

/* ============================ 板载按键 ============================== */
#define BOARD_KEY_PORT GPIOA
#define BOARD_KEY_PIN  GPIO_PIN_15 /* 用户按键(PA15), 按下为低电平 */

/**
 * @brief 把用户按键配置为带上拉的输入
 */
void BoardKeyInit(void);

/**
 * @brief 读取用户按键
 * @return 1: 按下, 0: 松开
 */
uint8_t BoardKeyPressed(void);

#endif /* BSP_BOARD_H */
