#ifndef DMMOTOR_H
#define DMMOTOR_H
#include "bsp_can.h"
#include "controller.h"
#include "daemon.h"
#include "motor_def.h"
#include <stdint.h>

#define DM_MOTOR_CNT 4

#define DM_P_MIN (-12.5f)
#define DM_P_MAX 12.5f
#define DM_V_MIN (-45.0f)
#define DM_V_MAX 45.0f
#define DM_T_MIN (-54.0f)
#define DM_T_MAX 54.0f

typedef enum
{
    DM_STATE_DISABLE = 0x00,        // 失能状态
    DM_STATE_ENABLE = 0x01,         // 使能状态
    DM_STATE_OVER_VOLT = 0x08,      // 超压状态
    DM_STATE_UNDER_VOLT = 0x09,     // 欠压状态
    DM_STATE_OVER_CURRENT = 0x0a,   // 过电流状态
    DM_STATE_MOS_OVER_TEMP = 0x0b,  // MOS过温状态
    DM_STATE_COIL_OVERTEMP = 0x0c,  // 电机线圈过温状态
    DM_STATE_COMM_LOSS = 0x0d,      // 通讯丢失
    DM_STATE_OVER_LOAD = 0x0e,      // 过载
} DMMotor_State_e;

typedef struct
{
    uint8_t id;
    DMMotor_State_e state;
    float velocity;
    float last_position;
    float position;
    float angle_single_round;
    float total_angle;
    float torque;
    float T_Mos;
    float T_Rotor;
    int32_t total_round;
} DM_Motor_Measure_s;

typedef struct
{
    uint16_t position_des;
    uint16_t velocity_des;
    uint16_t torque_des;
    uint16_t Kp;
    uint16_t Kd;
} DMMotor_Send_s;

typedef struct
{
    DM_Motor_Measure_s measure;              // 电机测量值
    Motor_Control_Setting_s motor_settings;  // 电机设置
    Motor_Controller_s motor_controller;     // 电机控制器

    CANInstance *motor_can_instace;  // 电机CAN实例

    Motor_Type_e motor_type;         // 电机类型
    Motor_Working_Type_e stop_flag;  // 电机工作状态

    DaemonInstance *motor_comm_daemon;   // 电机通讯守护进程
    DaemonInstance *motor_state_daemon;  // 电机状态守护进程
} DMMotorInstance;

typedef enum
{
    DM_CMD_MOTOR_MODE = 0xfc,     // 使能,会响应指令
    DM_CMD_RESET_MODE = 0xfd,     // 停止
    DM_CMD_ZERO_POSITION = 0xfe,  // 将当前的位置设置为编码器零位
    DM_CMD_CLEAR_ERROR = 0xfb     // 清除电机过热错误
} DMMotor_Mode_e;

DMMotorInstance *DMMotorInit(Motor_Init_Config_s *config);

void DMMotorSetRef(DMMotorInstance *motor, float ref);

void DMMotorOuterLoop(DMMotorInstance *motor, Closeloop_Type_e closeloop_type);

void DMMotorEnable(DMMotorInstance *motor);

void DMMotorStop(DMMotorInstance *motor);
void DMMotorCaliEncoder(DMMotorInstance *motor);
void DMMotorControlInit();
#endif  // !DMMOTOR