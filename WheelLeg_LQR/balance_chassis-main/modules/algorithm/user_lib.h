/**
 ******************************************************************************
 * @file	 user_lib.h
 * @author  Wang Hongxi
 * @version V1.0.0
 * @date    2021/2/18
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _USER_LIB_H
#define _USER_LIB_H

#include "stdint.h"
#include "main.h"
#include "cmsis_os.h"
#include "stm32f407xx.h"
#include "arm_math.h"


#ifndef user_malloc
#ifdef _CMSIS_OS_H
#define user_malloc pvPortMalloc
#else
#define user_malloc malloc
#endif
#endif

#define msin(x) (arm_sin_f32(x))
#define mcos(x) (arm_cos_f32(x))


/* boolean type definitions */
#ifndef TRUE
#define TRUE 1 /**< boolean true  */
#endif

#ifndef FALSE
#define FALSE 0 /**< boolean fails */
#endif

/* circumference ratio */
#ifndef PI
#define PI 3.14159265354f
#endif

#define VAL_LIMIT(val, min, max) \
    do                           \
    {                            \
        if ((val) <= (min))      \
        {                        \
            (val) = (min);       \
        }                        \
        else if ((val) >= (max)) \
        {                        \
            (val) = (max);       \
        }                        \
    } while (0)

#define ANGLE_LIMIT_360(val, angle)     \
    do                                  \
    {                                   \
        (val) = (angle) - (int)(angle); \
        (val) += (int)(angle) % 360;    \
    } while (0)

#define ANGLE_LIMIT_360_TO_180(val) \
    do                              \
    {                               \
        if ((val) > 180)            \
            (val) -= 360;           \
    } while (0)

#define VAL_MIN(a, b) ((a) < (b) ? (a) : (b))
#define VAL_MAX(a, b) ((a) > (b) ? (a) : (b))

typedef struct
{
    uint16_t order; // 拟合窗口的样本数
    uint32_t count; // 当前已有的样本数

    float *x; // 时间轴 (t) 缓冲区 (大小为 Order)
    float *y; // 信号值 (y) 缓冲区 (大小为 Order)

    float k; // 拟合结果: 斜率 (y = kx + b)
    float b; // 拟合结果: 截距 (y = kx + b)

    float t[4]; // 内部计算用的暂存器

    float StandardDeviation; // 拟合的标准差
} Ordinary_Least_Squares_s;

/**
 * @brief       最小二乘法(OLS)初始化
 * @param[in]   OLS: OLS结构体实例
 * @param[in]   order: 拟合窗口大小 (e.g. 5). 必须 > 2.
 */
void OLS_Init(Ordinary_Least_Squares_s *OLS, uint16_t order);

/**
 * @brief       (!! 已重构 !!) 最小二乘法(OLS)核心更新函数
 * @brief       这是*唯一*需要调用的OLS计算函数.
 * @param[in]   OLS: OLS结构体实例
 * @param[in]   deltax: 距上个样本的时间间隔 (dt)
 * @param[in]   y: 信号的新样本值
 */
void OLS_Update(Ordinary_Least_Squares_s *OLS, float deltax, float y);

/**
 * @brief       获取OLS计算出的导数 (斜率)
 * @param[in]   OLS: OLS结构体实例
 * @retval      返回斜率 k (导数)
 */
float OLS_Get_Derivative(Ordinary_Least_Squares_s *OLS);    

/**
 * @brief       获取OLS计算出的平滑值
 * @brief       (即: 拟合直线在*当前*时间点的值)
 * @param[in]   OLS: OLS结构体实例
 * @retval      返回平滑值 (k*x_now + b)
 */
float OLS_Get_Smooth(Ordinary_Least_Squares_s *OLS);

/**
 * @brief ???????????????,????????????????????????????
 *
 * @param size ?????��
 * @return void*
 */
void *
zmalloc(size_t size);

// 快速开方
float Sqrt(float x);
// 绝对限制
float abs_limit(float num, float Limit);
// 判断符号位
float sign(float value);
// 浮点死区
float float_deadband(float Value, float minValue, float maxValue);
//限幅函数
float float_constrain(float Value, float minValue, float maxValue);
//限幅函数
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//循环限幅函数
float loop_float_constrain(float Input, float minValue, float maxValue);
// 角度 °限幅 180 ~ -180
float theta_format(float Ang);

int float_rounding(float raw);

float *Norm3d(float *v);

float NormOf3d(float *v);

void Cross3d(float *v1, float *v2, float *res);

float Dot3d(float *v1, float *v2);

float AverageFilter(float new_data, float *buf, uint8_t len);

#define rad_format(Ang) loop_float_constrain((Ang), -PI, PI)

#endif
