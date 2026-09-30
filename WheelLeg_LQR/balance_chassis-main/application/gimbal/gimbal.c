#include "gimbal.h"
#include "bmi088.h"
#include "buzzer.h"
#include "can.h"
#include "controller.h"
#include "dji_motor.h"
#include "dmmotor.h"
#include "general_def.h"
#include "ins_task.h"
#include "message_center.h"
#include "motor_def.h"
#include "robot_def.h"

static INS_t *gimbal_IMU_data;  // 云台IMU数据
static DJIMotorInstance *yaw_motor;
static DMMotorInstance *pitch_motor;

static Publisher_t *gimbal_pub;                    // 云台应用消息发布者(云台反馈给cmd)
static Subscriber_t *gimbal_sub;                   // cmd控制消息订阅者
static Gimbal_Upload_Data_s gimbal_feedback_data;  // 回传给cmd的云台状态信息
static Gimbal_Ctrl_Cmd_s gimbal_cmd_recv;          // 来自cmd的控制信息

void GimbalInit()
{
    gimbal_IMU_data = INS_Init(GIMBAL_YAW_OFFSET_GYRO, GIMBAL_PITCH_OFFSET_GYRO,
                               GIMBAL_ROLL_OFFSET_GYRO);  // IMU先初始化,获取姿态数据指针赋给yaw电机的其他数据来源
    // YAW
    Motor_Init_Config_s yaw_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 2,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.25,    // 1.0
                .Ki = 0.0,    // 0.8
                .Kd = 0.05,    // 0.005
                .CoefA = 0.0, // 4.0
                .CoefB = 0.0, // 0.5
                .Output_LPF_RC = 0.1f,
                .DeadBand = 0.0,        // 0
                .Derivative_LPF_RC = 0.05, // 0.01
                .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter | PID_DerivativeFilter,
                .IntegralLimit = 4.0,
                .MaxOut = 20,
            },
            .speed_PID = {
                .Kp = 17000, // 17000
                .Ki = 0,     //
                .Kd = 0,
                // .CoefA = 0.8,
                // .CoefB = 0.1,
                .Output_LPF_RC = 0, // 0
                .Derivative_LPF_RC = 0,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_OutputFilter | PID_DerivativeFilter,
                .IntegralLimit = 0,
                .MaxOut = 16384,
            },
            .other_angle_feedback_ptr = &gimbal_IMU_data->YawTotalAngle,
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = &gimbal_IMU_data->Gyro[Z],
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedforward_flag = SPEED_FEEDFORWARD,
        },
        .motor_type = GM6020_CURRENT,
    };
    // PITCH
    Motor_Init_Config_s pitch_config = {
        .can_init_config = {
            .can_handle = &hcan1,
            .tx_id = 0x01,
            .rx_id = 0x11,
        },
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 0.55f,               // 1
                .Ki = 0.0,             // 0.15
                .Kd = 0.0,             // 0.006
                .CoefA = 0.0,          // 0.5
                .CoefB = 0.0,          // 0.6
                .DeadBand = 0.0,       // 0.005
                .Output_LPF_RC = 0, // 0.01
                .Derivative_LPF_RC = 0, // 0.01
                .Improve = PID_Trapezoid_Intergral | PID_ChangingIntegrationRate | PID_Integral_Limit | PID_OutputFilter | PID_DerivativeFilter | PID_Derivative_On_Measurement,
                .IntegralLimit = 1, // 1
                .MaxOut = 20,     // 600
            },
            .speed_PID = {
                .Kp = 1.4f,            // 10000
                .Ki = 0,              // 0
                .Kd = 0.0,            // 0
                .CoefA = 0,           // 0
                .CoefB = 0,           // 0
                .Output_LPF_RC = 0.0, // 0
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_OutputFilter | PID_Derivative_On_Measurement,
                .IntegralLimit = 3000, // 0
                .MaxOut = 10,           // 20000
            },
            .other_angle_feedback_ptr = (&gimbal_IMU_data->Pitch),
            // 还需要增加角速度额外反馈指针,注意方向,ins_task.md中有c板的bodyframe坐标系说明
            .other_speed_feedback_ptr = (&gimbal_IMU_data->Gyro[Y]),
        },
        .controller_setting_init_config = {
            .angle_feedback_source = OTHER_FEED,
            .speed_feedback_source = OTHER_FEED,
            .outer_loop_type = ANGLE_LOOP,
            .close_loop_type = ANGLE_LOOP | SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .feedback_reverse_flag = FEEDBACK_DIRECTION_REVERSE,
        },
        .motor_type = DM4310,
    };
    // 电机对total_angle闭环,上电时为零,会保持静止,收到遥控器数据再动
    yaw_motor = DJIMotorInit(&yaw_config);
    pitch_motor = DMMotorInit(&pitch_config);

    gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
}

/* 机器人云台控制核心任务,后续考虑只保留IMU控制,不再需要电机的反馈 */
void GimbalTask()
{
    BuzzerOn();
    // 获取云台控制数据
    // 后续增加未收到数据的处理
    SubGetMessage(gimbal_sub, &gimbal_cmd_recv);
    // @todo:现在已不再需要电机反馈,实际上可以始终使用IMU的姿态数据来作为云台的反馈,yaw电机的offset只是用来跟随底盘
    // 根据控制模式进行电机反馈切换和过渡,视觉模式在robot_cmd模块就已经设置好,gimbal只看yaw_ref和pitch_ref

    // AngleMutaionSolve(&gimbal_cmd_recv.yaw, gimbal_IMU_data->Yaw);

    switch (gimbal_cmd_recv.gimbal_mode)
    {
        // 停止
        case GIMBAL_ZERO_FORCE:
            DJIMotorStop(yaw_motor);
            DMMotorStop(pitch_motor);
            break;
        // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
        case GIMBAL_GYRO_MODE:  // 后续只保留此模式
                                // DJIMotorSetFeedfoward(yaw_motor,SPEED_FEEDFORWARD);
            DJIMotorEnable(yaw_motor);
            DMMotorEnable(pitch_motor);
            DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw);  // yaw和pitch会在robot_cmd中处理好多圈和单圈
            DMMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
            break;
        // 云台自由模式,使用编码器反馈,底盘和云台分离,仅云台旋转,一般用于调整云台姿态(英雄吊射等)/能量机关
        case GIMBAL_FREE_MODE:  // 后续删除,或加入云台追地盘的跟随模式(响应速度更快)
            // DJIMotorEnable(yaw_motor);
            // DJIMotorEnable(pitch_motor);
            // DJIMotorChangeFeed(yaw_motor, ANGLE_LOOP, OTHER_FEED);
            // DJIMotorChangeFeed(yaw_motor, SPEED_LOOP, OTHER_FEED);
            // DJIMotorChangeFeed(pitch_motor, ANGLE_LOOP, OTHER_FEED);
            // DJIMotorChangeFeed(pitch_motor, SPEED_LOOP, OTHER_FEED);
            // DJIMotorSetRef(yaw_motor, gimbal_cmd_recv.yaw); // yaw和pitch会在robot_cmd中处理好多圈和单圈
            // DJIMotorSetRef(pitch_motor, gimbal_cmd_recv.pitch);
            break;
        default:
            break;
    }

    // 在合适的地方添加pitch重力补偿前馈力矩
    // 根据IMU姿态/pitch电机角度反馈计算出当前配重下的重力矩
    // ...

    // 设置反馈数据,主要是imu和yaw的ecd
    gimbal_feedback_data.gimbal_imu_data = *gimbal_IMU_data;
    gimbal_feedback_data.yaw_motor_single_round_angle = yaw_motor->measure.angle_single_round;

    // 推送消息
    PubPushMessage(gimbal_pub, (void *)&gimbal_feedback_data);
}