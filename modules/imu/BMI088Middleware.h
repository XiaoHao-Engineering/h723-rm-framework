#ifndef BMI088MIDDLEWARE_H
#define BMI088MIDDLEWARE_H

#include "main.h"

#define BMI088_USE_SPI
//#define BMI088_USE_IIC

/* MC02 的 BMI088 片选: 加速度计 CS = PC0, 陀螺仪 CS = PC3 (CubeMX 中的标签见 main.h) */
#define CS1_ACCEL_GPIO_Port ACCEL_CS_GPIO_Port
#define CS1_ACCEL_Pin       ACCEL_CS_Pin
#define CS1_GYRO_GPIO_Port  GYRO_CS_GPIO_Port
#define CS1_GYRO_Pin        GYRO_CS_Pin


#if defined(BMI088_USE_SPI)
extern void BMI088_ACCEL_NS_L(void);
extern void BMI088_ACCEL_NS_H(void);

extern void BMI088_GYRO_NS_L(void);
extern void BMI088_GYRO_NS_H(void);

extern uint8_t BMI088_read_write_byte(uint8_t reg);

extern SPI_HandleTypeDef *BMI088_SPI;

#elif defined(BMI088_USE_IIC)

#endif

#endif
