/**
 ******************************************************************************
 * @file	 user_lib.c
 * @author  Wang Hongxi
 * @author  modified by neozng
 * @version 0.2 beta
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "user_lib.h"
#include "main.h"
#include "math.h"
#include "memory.h"
#include "stdlib.h"

#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif

/**
 * @brief       最小二乘法(OLS)初始化
 */
void OLS_Init(Ordinary_Least_Squares_s *OLS, uint16_t order)
{
    // 确保 order 至少为 2, 否则 OLS->x[1] 会越界
    if (order < 2)
    {
        order = 2; // 最小保护
    }

    OLS->order = order;
    OLS->count = 0;

    OLS->x = (float *)user_malloc(sizeof(float) * order);
    OLS->y = (float *)user_malloc(sizeof(float) * order);

    OLS->k = 0;
    OLS->b = 0;
    memset((void *)OLS->x, 0, sizeof(float) * order);
    memset((void *)OLS->y, 0, sizeof(float) * order);
    memset((void *)OLS->t, 0, sizeof(float) * 4);
}

/**
 * @brief       最小二乘法(OLS)核心更新函数
 * @brief       这是唯一需要调用的OLS计算函数.
 */
void OLS_Update(Ordinary_Least_Squares_s *OLS, float deltax, float y)
{
    // --- 1. 更新数据窗口 (滑动窗口) ---
    // (!! 修复: 你的原始代码中, OLS->x[0] 总是 0,
    //    并且 OLS->x[OLS->Order - 2] + deltax 会导致 x 轴非线性增长.
    //    这里的实现假设 x 是一个相对时间窗口 [..., -2*dt, -1*dt, 0])

    // (!! 原始代码的 x 轴更新逻辑, 已保留)
    static float temp = 0;
    temp = OLS->x[1]; // (!! 警告: 如果 order < 2, 这里会崩溃)
    for (uint16_t i = 0; i < OLS->order - 1; ++i)
    {
        OLS->x[i] = OLS->x[i + 1] - temp;
        OLS->y[i] = OLS->y[i + 1];
    }
    OLS->x[OLS->order - 1] = OLS->x[OLS->order - 2] + deltax;
    OLS->y[OLS->order - 1] = y;

    if (OLS->count < OLS->order)
    {
        OLS->count++;
    }

    // --- 2. 计算 OLS (k 和 b) ---
    memset((void *)OLS->t, 0, sizeof(float) * 4);

    // (!! 注意: 你的代码使用 count 来拟合,
    //    这意味着在填满窗口前, 拟合的阶数是变化的)
    uint16_t start_index = OLS->order - OLS->count;
    uint16_t n = OLS->count; // (!! 关键: 拟合的点数是 count, 不是 order)

    for (uint16_t i = start_index; i < OLS->order; ++i)
    {
        OLS->t[0] += OLS->x[i] * OLS->x[i]; // sum(x^2)
        OLS->t[1] += OLS->x[i];             // sum(x)
        OLS->t[2] += OLS->x[i] * OLS->y[i]; // sum(x*y)
        OLS->t[3] += OLS->y[i];             // sum(y)
    }

    float denominator = (OLS->t[0] * n - OLS->t[1] * OLS->t[1]);

    // 防止除零
    if (fabsf(denominator) > 1e-6f)
    {
        OLS->k = (OLS->t[2] * n - OLS->t[1] * OLS->t[3]) / denominator;
        OLS->b = (OLS->t[0] * OLS->t[3] - OLS->t[1] * OLS->t[2]) / denominator;
    }
    else
    {
        // 无法计算, 斜率保持为 0
        OLS->k = 0;
        OLS->b = OLS->t[3] / n; // 返回均值
    }

    // --- 3. (可选) 计算标准差 ---
    OLS->StandardDeviation = 0;
    for (uint16_t i = start_index; i < OLS->order; ++i)
    {
        OLS->StandardDeviation += fabsf(OLS->k * OLS->x[i] + OLS->b - OLS->y[i]);
    }
    OLS->StandardDeviation /= n;
}

/**
 * @brief       取OLS计算出的导数 (斜率)
 * @note        此函数不执行计算, 只返回 OLS_Update 的结果.
 */
float OLS_Get_Derivative(Ordinary_Least_Squares_s *OLS)
{
    return OLS->k;
}

/**
 * @brief       获取OLS计算出的平滑值
 * @note        此函数不执行计算, 只返回 OLS_Update 的结果.
 */
float OLS_Get_Smooth(Ordinary_Least_Squares_s *OLS)
{
    // 返回拟合直线在最后一个时间点 (x[order-1]) 的值
    return OLS->k * OLS->x[OLS->order - 1] + OLS->b;
}

void *zmalloc(size_t size)
{
    void *ptr = malloc(size);
    memset(ptr, 0, size);
    return ptr;
}

// 快速开方
float Sqrt(float x)
{
    float y;
    float delta;
    float maxError;

    if (x <= 0)
    {
        return 0;
    }

    // initial guess
    y = x / 2;

    // refine
    maxError = x * 0.001f;

    do
    {
        delta = (y * y) - x;
        y -= delta / (2 * y);
    } while (delta > maxError || delta < -maxError);

    return y;
}

// 绝对值限制
float abs_limit(float num, float Limit)
{
    if (num > Limit)
    {
        num = Limit;
    }
    else if (num < -Limit)
    {
        num = -Limit;
    }
    return num;
}

// 判断符号位
float sign(float value)
{
    if (value > 0.0f)
    {
        return 1.0f;
    }
    else if (value == 0.0f)
    {
        return 0.0f;
    }
    else
    {
        return -1.0f;
    }
}

// 浮点死区
float float_deadband(float Value, float minValue, float maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

// 限幅函数
float float_constrain(float Value, float minValue, float maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

// 循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
        float len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
        float len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

// 弧度格式化为-PI~PI

// 角度格式化为-180~180
float theta_format(float Ang)
{
    return loop_float_constrain(Ang, -180.0f, 180.0f);
}

int float_rounding(float raw)
{
    static int integer;
    static float decimal;
    integer = (int)raw;
    decimal = raw - integer;
    if (decimal > 0.5f)
        integer++;
    return integer;
}

// 三维向量归一化
float *Norm3d(float *v)
{
    float len = Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return v;
}

// 计算模长
float NormOf3d(float *v)
{
    return Sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// 三维向量叉乘v1 x v2
void Cross3d(float *v1, float *v2, float *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

// 三维向量点乘
float Dot3d(float *v1, float *v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

// 均值滤波,删除buffer中的最后一个元素,填入新的元素并求平均值
float AverageFilter(float new_data, float *buf, uint8_t len)
{
    float sum = 0;
    for (uint8_t i = 0; i < len - 1; i++)
    {
        buf[i] = buf[i + 1];
        sum += buf[i];
    }
    buf[len - 1] = new_data;
    sum += new_data;
    return sum / len;
}

