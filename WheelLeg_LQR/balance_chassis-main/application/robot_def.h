/**
 * @file robot_def.h
 * @author NeoZeng neozng1@hnu.edu.cn
 * @author Even
 * @version 0.1
 * @date 2022-12-02
 *
 * @copyright Copyright (c) HNU YueLu EC 2022 all rights reserved
 *
 */
#pragma once  // 可以用#pragma once代替#ifndef ROBOT_DEF_H(header guard)
#include <stdint.h>
#ifndef ROBOT_DEF_H
#define ROBOT_DEF_H

#include "ins_task.h"
#include "master_process.h"
#include "stdint.h"

/* 开发板类型定义,烧录时注意不要弄错对应功能;修改定义后需要重新编译,只能存在一个定义! */
// #define ONE_BOARD // 单板控制整车
 #define CHASSIS_BOARD  // 底盘板
// #define GIMBAL_BOARD  // 云台板

#define VISION_USE_VCP  // 使用虚拟串口发送视觉数据
// #define VISION_USE_UART // 使用串口发送视觉数据

/* 机器人重要参数定义,注意根据不同机器人进行修改,浮点数需要以.0或f结尾,无符号以u结尾 */
// 云台参数
#define YAW_CHASSIS_ALIGN_ECD 6461   // 云台和底盘对齐指向相同方向时的电机编码器值,若对云台有机械改动需要修改
#define YAW_CHASSIS_SIDE_ECD 4440    // 云台和底盘侧面对齐时的电机编码器值,若对云台有机械改动需要修改
#define YAW_ECD_GREATER_THAN_4096 1  // ALIGN_ECD值是否大于4096,是为1,否为0;用于计算云台偏转角度
#define PITCH_HORIZON_ECD 3412       // 云台处于水平位置时编码器值,若对云台有机械改动需要修改
#define PITCH_MAX_ANGLE (12.0f)      // 云台竖直方向最大角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)
#define PITCH_MIN_ANGLE (-22.0f)     // 云台竖直方向最小角度 (注意反馈如果是陀螺仪，则填写陀螺仪的角度)

#define GIMBAL_YAW_OFFSET_GYRO 0  // 云台陀螺仪安装偏移对应坐标轴角度
#define GIMBAL_PITCH_OFFSET_GYRO 0
#define GIMBAL_ROLL_OFFSET_GYRO 180

#define CHASSIS_YAW_OFFSET_GYRO 90  // 底盘陀螺仪安装偏移对应坐标轴角度
#define CHASSIS_PITCH_OFFSET_GYRO 0
#define CHASSIS_ROLL_OFFSET_GYRO 90

// 发射参数
#define REDUCTION_RATIO_LOADER 36.0f  // 拨盘电机的减速比,英雄需要修改为3508的19.0f
#define NUM_PER_CIRCLE 8              // 拨盘一圈的装载量
#define LOAD_RATIO 1                  // 中心供弹需要机械减速比
#define ONE_BULLET_PER_SECOND                                                                                          \
    (REDUCTION_RATIO_LOADER * 60 / NUM_PER_CIRCLE)  // 一秒发射一发系数,乘上射频即为每秒发射的弹丸数量
#define ONE_BULLET_DELTA_ANGLE                                                                                         \
    (8192 * REDUCTION_RATIO_LOADER / NUM_PER_CIRCLE)  // 发射一发弹丸拨盘转动的距离,由机械设计图纸给出

#define BALANCE_MAX_SPEED 2.5f  // 底盘最大速度,单位m/s
// 检查是否出现主控板定义冲突,只允许一个开发板定义存在,否则编译会自动报错
#if (defined(ONE_BOARD) && defined(CHASSIS_BOARD)) || (defined(ONE_BOARD) && defined(GIMBAL_BOARD)) ||                 \
    (defined(CHASSIS_BOARD) && defined(GIMBAL_BOARD))
#error Conflict board definition! You can only define one board type.
#endif

#pragma pack(1)  // 压缩结构体,取消字节对齐,下面的数据都可能被传输
/* -------------------------基本控制模式和数据类型定义-------------------------*/
/**
 * @brief 这些枚举类型和结构体会作为CMD控制数据和各应用的反馈数据的一部分
 *
 */
// 机器人状态
typedef enum
{
    ROBOT_STOP = 0,
    ROBOT_READY,
} Robot_Status_e;

// 应用状态
typedef enum
{
    APP_OFFLINE = 0,
    APP_ONLINE,
    APP_ERROR,
} App_Status_e;

// UI模式设置
typedef enum
{
    UI_KEEP = 0,
    UI_REFRESH,
} ui_mode_e;

// 底盘模式设置
/**
 * @brief 后续考虑修改为云台跟随底盘,而不是让底盘去追云台,云台的惯量比底盘小.
 *
 */
typedef enum
{
    CHASSIS_ZERO_FORCE = 0,     // 电流零输入
    CHASSIS_RESET,              // 底盘重置,双腿缩回
    CHASSIS_FOLLOW_GIMBAL_YAW,  // 跟随模式，底盘叠加角度环控制
    CHASSIS_ROTATE,             // 小陀螺模式
    CHASSIS_ROTATE_REVERSE,
} chassis_mode_e;

// 云台模式设置
typedef enum
{
    GIMBAL_ZERO_FORCE = 0,  // 电流零输入
    GIMBAL_FREE_MODE,  // 云台自由运动模式,即与底盘分离(底盘此时应为NO_FOLLOW)反馈值为电机total_angle;似乎可以改为全部用IMU数据?
    GIMBAL_GYRO_MODE,  // 云台陀螺仪反馈模式,反馈值为陀螺仪pitch,total_yaw_angle,底盘可以为小陀螺和跟随模式
} gimbal_mode_e;

// 发射模式设置
typedef enum
{
    SHOOT_OFF = 0,
    SHOOT_ON,
} shoot_mode_e;
typedef enum
{
    FRICTION_OFF = 0,  // 摩擦轮关闭
    FRICTION_ON,       // 摩擦轮开启
} friction_mode_e;

typedef enum
{
    LID_OPEN = 0,  // 弹舱盖打开
    LID_CLOSE,     // 弹舱盖关闭
} lid_mode_e;

typedef enum
{
    LOAD_STOP = 0,   // 停止发射
    LOAD_REVERSE,    // 反转
    LOAD_1_BULLET,   // 单发
    LOAD_3_BULLET,   // 三发
    LOAD_BURSTFIRE,  // 连发
} loader_mode_e;

typedef enum
{
    VISION_IDLE = 0,
    VISION_AUTOAIM,
    VISION_SMALL_BUFF,
    VISION_BIG_BUFF,
} vision_mode_e;

typedef enum
{
    CHASSIS_ALIGN = 0,
    CHASSIS_SIDLE,
} chassis_direction_e;

typedef struct
{
    float chassis_power_mx;
} Chassis_Power_Data_s;

typedef enum
{
    CHASSIS_JUMP_OFF = 0,
    CHASSIS_JUMP_ON,
} chassis_jump_mode_e;

typedef enum
{
    CHASSIS_LEGLEN_NO_CONTROL = 0,
    CHASSIS_LEGLEN_MID,
    CHASSIS_LEGLEN_HIGH,
} chassis_leglen_mode_e;

typedef enum
{
    CHASSIS_MOTION_SLOW = 0,
    CHASSIS_MOTION_FAST,
} chassis_motion_mode_e;

typedef enum
{
    CAP_OFF = 0,  // 超级电容关闭
    CAP_ON,       // 超级电容开启
} cap_mode_e;

/* ----------------CMD应用发布的控制数据,应当由gimbal/chassis/shoot订阅---------------- */
/**
 * @brief 对于双板情况,遥控器和pc在云台,裁判系统在底盘
 *
 */
// cmd发布的底盘控制数据,由chassis订阅
typedef struct
{
    // 控制部分
    float vx;           // 前进方向速度
    float vy;           // 侧方方向速度
    float target_roll;  // 目标roll角度(可以用来roll轴小陀螺,以及小黑子之舞)

    int8_t delta_leglen;  // 腿长
    float offset_angle;   // 底盘和归中位置的夹角

    chassis_mode_e chassis_mode;
    chassis_direction_e direction;
    chassis_jump_mode_e jump_mode;      // 跳跃模式
    chassis_leglen_mode_e leglen_mode;  // 腿长模式
    chassis_motion_mode_e motion_mode;  // 运动速度

    // UI部分
    friction_mode_e friction_mode;  //  摩擦轮状态
    vision_mode_e vision_mode;      //  视觉状态
    ui_mode_e ui_mode;              //  UI状态
    loader_mode_e loader_mode;      //  拨盘状态

} Chassis_Ctrl_Cmd_s;

// cmd发布的云台控制数据,由gimbal订阅
typedef struct
{  // 云台角度控制
    float yaw;
    float pitch;

    gimbal_mode_e gimbal_mode;
} Gimbal_Ctrl_Cmd_s;

// cmd发布的发射控制数据,由shoot订阅
typedef struct
{
    shoot_mode_e shoot_mode;
    loader_mode_e load_mode;
    lid_mode_e lid_mode;
    friction_mode_e friction_mode;
    uint16_t rest_heat;
    float bullet_speed;
    uint16_t bullet_cnt;
    float shoot_rate;  // 连续发射的射频,unit per s,发/秒
} Shoot_Ctrl_Cmd_s;

/* ----------------gimbal/shoot/chassis发布的反馈数据----------------*/
/**
 * @brief 由cmd订阅,其他应用也可以根据需要获取.
 *
 */

typedef struct
{
#if defined(CHASSIS_BOARD) || defined(GIMBAL_BOARD)  // 非单板的时候底盘还将imu数据回传(若有必要)
    // attitude_t chassis_imu_data;
#endif
    int16_t rest_heat;               // 剩余枪口热量
    uint16_t shooter_cooling_value;  // 枪口冷却速率
    uint16_t bullet_cnt;             // 当前发弹数量
    float bullet_speed;              // 上一次枪口射速
} Chassis_Upload_Data_s;

typedef struct
{
    INS_t gimbal_imu_data;
    uint16_t yaw_motor_single_round_angle;
    float pitch_motor_ecd;
} Gimbal_Upload_Data_s;

typedef struct
{
    // code to go here
    // ...
} Shoot_Upload_Data_s;

#pragma pack()  // 开启字节对齐,结束前面的#pragma pack(1)

#endif  // !ROBOT_DEF_H