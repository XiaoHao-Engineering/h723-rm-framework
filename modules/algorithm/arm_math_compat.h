/**
 * @file arm_math_compat.h
 * @brief 极简的 CMSIS-DSP 矩阵接口替代实现(只包含框架用到的几个函数)
 *
 * 为什么要这个文件:
 *   F4 版本链接了 ARM 官方的 libCMSISDSP.a(基于 arm_math.h). H7 工程里没有
 *   预编译的 CM7 版本 DSP 库, 而框架只用到 sin/cos 和少量矩阵运算, 因此这里给出
 *   一个与 arm_math.h 中同名同签名的轻量实现, module 层的代码不需要任何修改.
 *
 * 如果你后面要用 FFT/滤波器/CMSIS-NN 等完整 DSP 功能, 请从 ARM 官网或
 * STM32CubeMX 的 X-CUBE-ALGOBUILD 中引入完整的 CMSIS-DSP 源码, 并删除本文件.
 */

#ifndef ARM_MATH_COMPAT_H
#define ARM_MATH_COMPAT_H

#include <stdint.h>
#include <math.h>
#include <string.h> /* arm_math.h 原本也会引入 string.h, module 层有直接用 memcpy/memset 的地方 */

/* arm_math.h 用 ARM_MATH_H 作为头文件保护宏, 若已包含真正的 CMSIS-DSP 则不再重复定义 */
#ifndef ARM_MATH_H
typedef float float32_t;
#endif

/* 与 CMSIS-DSP 一致的矩阵实例结构体 */
typedef struct
{
    uint16_t numRows;  // 行数
    uint16_t numCols;  // 列数
    float32_t *pData;  // 数据(行优先连续存放)
} arm_matrix_instance_f32;

typedef enum
{
    ARM_MATH_SUCCESS = 0,
    ARM_MATH_ARGUMENT_ERROR = -1,
    ARM_MATH_LENGTH_ERROR = -2,
    ARM_MATH_SIZE_MISMATCH = -3,
    ARM_MATH_NANINF = -4,
    ARM_MATH_SINGULAR = -5,
} arm_status;

/* 三角函数: H7 有硬件 FPU, 直接用 libm 即可 */
#ifndef arm_sin_f32
#define arm_sin_f32(x) sinf(x)
#endif
#ifndef arm_cos_f32
#define arm_cos_f32(x) cosf(x)
#endif

arm_status arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nColumns, float32_t *pData);
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst);
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst);
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst);

#endif /* ARM_MATH_COMPAT_H */
