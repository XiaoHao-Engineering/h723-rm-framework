#ifndef GENERAL_DEF_H
#define GENERAL_DEF_H

// 一些module的通用数值型定义,注意条件macro兼容,一些宏可能在math.h中已经定义过了

#ifndef PI
#define PI 3.1415926535f
#endif
#define PI2 (PI * 2.0f) // 2 pi

#define RAD_2_DEGREE 57.2957795f    // 180/pi
#define DEGREE_2_RAD 0.01745329252f // pi/180

#define RPM_2_ANGLE_PER_SEC 6.0f       // ×360°/60sec
#define RPM_2_RAD_PER_SEC 0.104719755f // ×2pi/60sec

/* ---------------------------------------------------------------------------
 * H7 带硬件双精度 FPU, 求 sin/cos 直接用 libm 就够了, 不需要再链接 CMSIS-DSP.
 * 为了不改动 module 层代码, 这里用宏提供与 CMSIS-DSP 同名的接口.
 * 若你确实需要 CMSIS-DSP(FFT/矩阵等), 请自行在工程中加入对应的 CMSIS-DSP 源文件/库.
 * ------------------------------------------------------------------------- */
#include <math.h>

#ifndef arm_cos_f32
#define arm_cos_f32(x) cosf(x)
#endif
#ifndef arm_sin_f32
#define arm_sin_f32(x) sinf(x)
#endif

#endif // !GENERAL_DEF_H