/**
 * @file arm_math_compat.c
 * @brief CMSIS-DSP 矩阵函数的轻量实现(仅框架用到的部分)
 */

#include "arm_math_compat.h"

#include <string.h>

arm_status arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows, uint16_t nColumns, float32_t *pData)
{
    if (S == NULL)
        return ARM_MATH_ARGUMENT_ERROR;

    S->numRows = nRows;
    S->numCols = nColumns;
    S->pData = pData;
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst)
{
    if (pSrcA->numRows != pSrcB->numRows || pSrcA->numCols != pSrcB->numCols ||
        pSrcA->numRows != pDst->numRows || pSrcA->numCols != pDst->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    uint32_t cnt = (uint32_t)pSrcA->numRows * pSrcA->numCols;
    for (uint32_t i = 0; i < cnt; ++i)
        pDst->pData[i] = pSrcA->pData[i] + pSrcB->pData[i];

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst)
{
    if (pSrcA->numRows != pSrcB->numRows || pSrcA->numCols != pSrcB->numCols ||
        pSrcA->numRows != pDst->numRows || pSrcA->numCols != pDst->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    uint32_t cnt = (uint32_t)pSrcA->numRows * pSrcA->numCols;
    for (uint32_t i = 0; i < cnt; ++i)
        pDst->pData[i] = pSrcA->pData[i] - pSrcB->pData[i];

    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst)
{
    if (pSrcA->numCols != pSrcB->numRows || pDst->numRows != pSrcA->numRows || pDst->numCols != pSrcB->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    for (uint16_t i = 0; i < pSrcA->numRows; ++i)
    {
        for (uint16_t j = 0; j < pSrcB->numCols; ++j)
        {
            float32_t sum = 0.0f;
            for (uint16_t k = 0; k < pSrcA->numCols; ++k)
                sum += pSrcA->pData[i * pSrcA->numCols + k] * pSrcB->pData[k * pSrcB->numCols + j];
            pDst->pData[i * pDst->numCols + j] = sum;
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst)
{
    if (pSrc->numRows != pDst->numCols || pSrc->numCols != pDst->numRows)
        return ARM_MATH_SIZE_MISMATCH;

    for (uint16_t i = 0; i < pSrc->numRows; ++i)
        for (uint16_t j = 0; j < pSrc->numCols; ++j)
            pDst->pData[j * pDst->numCols + i] = pSrc->pData[i * pSrc->numCols + j];

    return ARM_MATH_SUCCESS;
}

/**
 * @brief 高斯-约当消元法求逆矩阵(方阵)
 */
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst)
{
    uint16_t n = pSrc->numRows;

    if (pSrc->numRows != pSrc->numCols || n != pDst->numRows || n != pDst->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    /* 为了支持原地求逆(pSrc == pDst), 先拷贝到临时数组 */
#define ARM_MAT_MAX_DIM 12
    static float32_t a[ARM_MAT_MAX_DIM][2 * ARM_MAT_MAX_DIM];
    static float32_t res[ARM_MAT_MAX_DIM][ARM_MAT_MAX_DIM];

    if (n > ARM_MAT_MAX_DIM)
        return ARM_MATH_LENGTH_ERROR;

    for (uint16_t i = 0; i < n; ++i)
        for (uint16_t j = 0; j < n; ++j)
        {
            a[i][j] = pSrc->pData[i * n + j];
            a[i][j + n] = (i == j) ? 1.0f : 0.0f;
        }

    for (uint16_t col = 0; col < n; ++col)
    {
        /* 选主元 */
        uint16_t pivot = col;
        float32_t max_val = fabsf(a[col][col]);
        for (uint16_t r = col + 1; r < n; ++r)
        {
            if (fabsf(a[r][col]) > max_val)
            {
                max_val = fabsf(a[r][col]);
                pivot = r;
            }
        }

        if (max_val < 1e-12f)
            return ARM_MATH_SINGULAR;

        if (pivot != col)
        {
            for (uint16_t j = 0; j < 2 * n; ++j)
            {
                float32_t tmp = a[col][j];
                a[col][j] = a[pivot][j];
                a[pivot][j] = tmp;
            }
        }

        float32_t inv = 1.0f / a[col][col];
        for (uint16_t j = 0; j < 2 * n; ++j)
            a[col][j] *= inv;

        for (uint16_t r = 0; r < n; ++r)
        {
            if (r == col)
                continue;
            float32_t factor = a[r][col];
            if (factor == 0.0f)
                continue;
            for (uint16_t j = 0; j < 2 * n; ++j)
                a[r][j] -= factor * a[col][j];
        }
    }

    for (uint16_t i = 0; i < n; ++i)
        for (uint16_t j = 0; j < n; ++j)
            res[i][j] = a[i][j + n];

    memcpy(pDst->pData, res, sizeof(float32_t) * n * n);
    return ARM_MATH_SUCCESS;
}
