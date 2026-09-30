/**
 ******************************************************************************
 * @file	 controller.h
 * @author  Wang Hongxi
 * @version V1.1.3
 * @date    2021/7/3
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "arm_math.h"
#include "bsp_dwt.h"
#include "main.h"
#include "memory.h"
#include "stdint.h"
#include "stdlib.h"
#include <math.h>
#include "user_lib.h"

#ifndef abs
#define abs(x) ((x > 0) ? x : -x)
#endif

// PID 优化环节使能标志位,通过位与可以判断启用的优化环节;也可以改成位域的形式
typedef enum
{
    PID_IMPROVE_NONE = 0b00000000,                // 0000 0000
    PID_Integral_Limit = 0b00000001,              // 0000 0001
    PID_Derivative_On_Measurement = 0b00000010,   // 0000 0010
    PID_Trapezoid_Intergral = 0b00000100,         // 0000 0100
    PID_Proportional_On_Measurement = 0b00001000, // 0000 1000
    PID_OutputFilter = 0b00010000,                // 0001 0000
    PID_ChangingIntegrationRate = 0b00100000,     // 0010 0000
    PID_DerivativeFilter = 0b01000000,            // 0100 0000
    PID_ErrorHandle = 0b10000000,                 // 1000 0000
} PID_Improvement_e;

/* PID 报错类型枚举*/
typedef enum errorType_e
{
    PID_ERROR_NONE = 0x00U,
    PID_MOTOR_BLOCKED_ERROR = 0x01U
} ErrorType_e;

typedef struct
{
    uint64_t ERRORCount;
    ErrorType_e ERRORType;
} PID_ErrorHandler_t;

/* PID结构体 */
typedef struct
{
    //---------------------------------- init config block
    // config parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;
    float DeadBand;

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit;     // 积分限幅
    float CoefA;             // 变速积分 For Changing Integral
    float CoefB;             // 变速积分 ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC;     // 输出滤波器 RC = 1/omegac
    float Derivative_LPF_RC; // 微分滤波器系数

    //-----------------------------------
    // for calculating
    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
    float Last_ITerm;

    float Pout;
    float Iout;
    float Dout;
    float ITerm;

    float Output;
    float Last_Output;
    float Last_Dout;

    float Ref;

    uint32_t DWT_CNT;
    float dt;

    PID_ErrorHandler_t ERRORHandler;
} PIDInstance;

/* 用于PID初始化的结构体*/
typedef struct // config parameter
{
    // basic parameter
    float Kp;
    float Ki;
    float Kd;
    float MaxOut;   // 输出限幅
    float DeadBand; // 死区

    // improve parameter
    PID_Improvement_e Improve;
    float IntegralLimit; // 积分限幅
    float CoefA;         // AB为变速积分参数,变速积分实际上就引入了积分分离
    float CoefB;         // ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
    float Output_LPF_RC; // RC = 1/omegac
    float Derivative_LPF_RC;
} PID_Init_Config_s;

typedef struct
{
    // --- 模型参数 (G(s) = c[2]s^2 + c[1]s + c[0]) ---
    // (c[0] = Kv (速度增益), c[1] = Ka (加速度增益))
    // (c[2] 通常为 0, 除非你需要Jerk(加加速度)前馈)
    float c[3];

    // --- 信号处理 ---
    float Ref;      // 1. [输入] 原始目标信号 (e.g. 原始角度/速度)
    float LPF_Ref;  // 2. [LPF] 经过低通滤波后的目标
    float Ref_dot;  // 3. [导数] LPF_Ref 的一阶导数 (速度)
    float Ref_ddot; // 4. [导数] Ref_dot 的二阶导数 (加速度)
    float Output;   // 5. [输出] 最终的前馈力矩

    // --- 历史值 ---
    float Last_LPF_Ref;
    float Last_Ref_dot;

    // --- 配置 ---
    float MaxOut; // 最大输出限制
    float LPF_RC; // LPF 的 RC 时间常数

    // --- OLS 实例 ---
    uint16_t Ref_dot_OLS_Order;           // 速度OLS的窗口大小
    Ordinary_Least_Squares_s Ref_dot_OLS; // OLS实例 (用于 Ref -> Ref_dot)

    uint16_t Ref_ddot_OLS_Order;           // 加速度OLS的窗口大小
    Ordinary_Least_Squares_s Ref_ddot_OLS; // OLS实例 (用于 Ref_dot -> Ref_ddot)

    // --- 计时 ---
    uint32_t DWT_CNT;
    float dt;
} Feedforward_s;

/**
 * @brief 初始化PID实例
 * @todo 待修改为统一的PIDRegister风格
 * @param pid    PID实例指针
 * @param config PID初始化配置
 */
void PIDInit(PIDInstance *pid, PID_Init_Config_s *config);

/**
 * @brief 计算PID输出
 *
 * @param pid     PID实例指针
 * @param measure 反馈值
 * @param ref     设定值
 * @return float  PID计算输出
 */
float PIDCalculate(PIDInstance *pid, float measure, float ref);

/**
 * @brief       OLS前馈控制器初始化
 * @param[in]   ffc: 前馈控制器实例
 * @param[in]   max_out: 最大输出限制
 * @param[in]   c: 物理模型增益 [c0=Kv, c1=Ka, c2=Kj]
 * @param[in]   lpf_rc: LPF时间常数 (s)
 * @param[in]   ref_dot_ols_order: 速度OLS的窗口大小 (e.g. 5)
 * @param[in]   ref_ddot_ols_order: 加速度OLS的窗口大小 (e.g. 5)
 */
void Feedforward_Init(
    Feedforward_s *ffc,
    float max_out,
    float *c,
    float lpf_rc,
    uint16_t ref_dot_ols_order,
    uint16_t ref_ddot_ols_order);

/**
 * @brief       OLS前馈控制器计算
 * @param[in]   ffc: 前馈控制器实例
 * @param[in]   ref: *原始* 目标值 (e.g. 角度或速度)
 * @retval      返回计算出的前馈力矩
 */
float Feedforward_Calculate(Feedforward_s *ffc, float ref);

#endif