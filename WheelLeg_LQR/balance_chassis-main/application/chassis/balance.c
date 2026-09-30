#include "balance.h"
#include <math.h>
#include "can.h"
#include "dji_motor.h"
#include "fly_detection.h"
#include "general_def.h"
#include "ins_task.h"
#include "linkNleg.h"
#include "lqr_calc.h"
#include "motor_def.h"
#include "rm_referee.h"
#include "robot_def.h"
#include "speed_estimation.h"
// module
#include "LK9025.h"
#include "buzzer.h"
#include "can_comm.h"
#include "controller.h"
#include "dmmotor.h"
#include "referee_task.h"
#include "super_cap.h"
#include "stp23.h"
#include "usart.h"
#include "user_lib.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"

#include "arm_math.h"  // 需要用到较多三角函数
#include "stdbool.h"
#include "stdint.h"

// 计时变量
static uint32_t balance_dwt_cnt;
static float del_t;

// 底盘拥有的实例模块
static INS_t *Chassis_IMU_data;
static Chassis_Ctrl_Cmd_s chassis_cmd_recv;
static Chassis_Upload_Data_s chassis_feedback_data;  // 底盘反馈数据
// 四个关节电机和两个驱动轮电机
static DMMotorInstance *lf, *lb, *rf, *rb, *joint[4];  // 指针数组方便传参和调试
static DJIMotorInstance *l_driven, *r_driven, *driven[2];

// 两个腿的参数,0为左腿,1为右腿
static LinkNPodParam l_side, r_side;
static ChassisParam chassis;

static PIDInstance leg_length_pid_l, leg_length_pid_r;  // 腿长PID控制器
static PIDInstance leg_speed_pid_l, leg_speed_pid_r;    // 腿速PID控制器
// 综合运动补偿的PID控制器
static PIDInstance roll_compensation_pid;
static PIDInstance steer_p_pid, steer_v_pid;  // 转向PID,有转向指令时使用IMU的加速度反馈积分以获取速度和位置状态量
static PIDInstance anti_crash_pid;            // 抗劈叉,将输出以相反的方向叠加到左右腿的上

static referee_info_t *referee_data;        // 用于获取裁判系统的数据
static Referee_Interactive_info_t ui_data;  // UI数据，将底盘中的数据传入此结构体的对应变量中，UI会自动检测是否变化，对应显示UI

static CANCommInstance *cmd_can_comm;  // 底盘CAN通信实例
SuperCapInstance *cap;                 // 超级电容
float final_power_limit = 0;
float power_limit = 0;
static LidarInfo_s *lidar_info;

static PIDInstance charge_buffer_pid;
static PIDInstance motion_buffer_pid;
void BalanceInit()
{
    Chassis_IMU_data = INS_Init(CHASSIS_YAW_OFFSET_GYRO, CHASSIS_PITCH_OFFSET_GYRO, CHASSIS_ROLL_OFFSET_GYRO);

    // TODO:裁判系统&UI
    referee_data = UITaskInit(&huart6, &ui_data);

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x311,
            .rx_id = 0x312,
        },
        .daemon_count = 100,
        .recv_data_len = sizeof(Chassis_Ctrl_Cmd_s),
        .send_data_len = sizeof(Chassis_Upload_Data_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);
    // 驱动轮电机
    Motor_Init_Config_s driven_conf = {
        // 写一个,剩下的修改方向和id即可
        .can_init_config.can_handle = &hcan2,
        .controller_setting_init_config = {
            .outer_loop_type = CURRENT_LOOP,
            .close_loop_type = CURRENT_LOOP,
        },
        .controller_param_init_config = {
            .current_PID = {
                .Kp = 1,
                .Ki = 0,
                .Kd = 0,
                .Improve = PID_OutputFilter,
                .Output_LPF_RC = 0.01f,
                .MaxOut = 16384,
            },
        },
        .motor_type = M3508,
    };
    driven_conf.can_init_config.tx_id = 1;
    driven[LD] = l_driven = DJIMotorInit(&driven_conf);
    driven_conf.can_init_config.tx_id = 2;
    driven[RD] = r_driven = DJIMotorInit(&driven_conf);

    // 关节电机
    Motor_Init_Config_s joint_conf = {
        .can_init_config.can_handle = &hcan1,
        .controller_setting_init_config = {
            .outer_loop_type = TORQUE_LOOP,
            .close_loop_type = TORQUE_LOOP,
        },
        .controller_param_init_config = {
            .torque_PID = {
                .Kp = 1,
                .Ki = 0,
                .Kd = 0,
                .MaxOut = 54.0f,
            },
        },
        .motor_type = DM8009};

    joint_conf.can_init_config.tx_id = 1;
    joint_conf.can_init_config.rx_id = 0x11;
    joint[LF] = lf = DMMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 2;
    joint_conf.can_init_config.rx_id = 0x12;
    joint[LB] = lb = DMMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 3;
    joint_conf.can_init_config.rx_id = 0x13;
    joint[RF] = rf = DMMotorInit(&joint_conf);
    joint_conf.can_init_config.tx_id = 4;
    joint_conf.can_init_config.rx_id = 0x14;
    joint[RB] = rb = DMMotorInit(&joint_conf);

    PID_Init_Config_s leg_length_pid_conf = {
        .Kp = 10,
        .Ki = 0,
        .Kd = 0,
        .MaxOut = 2,
        .DeadBand = 0.0001f,
        .Improve = PID_Trapezoid_Intergral | PID_Derivative_On_Measurement | PID_Integral_Limit,
    };
    PIDInit(&leg_length_pid_l, &leg_length_pid_conf);
    PIDInit(&leg_length_pid_r, &leg_length_pid_conf);

    PID_Init_Config_s leg_speed_pid_conf = {
        .Kp = 300,
        .Ki = 50,
        .Kd = 0,
        .MaxOut = 160,
        .IntegralLimit = 50,
        .Improve = PID_Trapezoid_Intergral | PID_Derivative_On_Measurement | PID_Integral_Limit,
    };
    PIDInit(&leg_speed_pid_l, &leg_speed_pid_conf);
    PIDInit(&leg_speed_pid_r, &leg_speed_pid_conf);

    // Roll补偿
    PID_Init_Config_s roll_compensation_pid_conf = {
        .Kp = 4000,
        .Ki = 0,
        .Kd = 0,
        .MaxOut = 400,
        .DeadBand = 0.0001f,
        .Improve = PID_Trapezoid_Intergral | PID_Derivative_On_Measurement | PID_Integral_Limit,
    };
    PIDInit(&roll_compensation_pid, &roll_compensation_pid_conf);

    // 航向控制
    // 角度环
    PID_Init_Config_s steer_p_pid_conf = {
        .Kp = 5,
        .Kd = 0,
        .Ki = 0.0f,
        .MaxOut = 3,
        .DeadBand = 0.0f,
        .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
    };
    PIDInit(&steer_p_pid, &steer_p_pid_conf);
    // 速度环
    PID_Init_Config_s steer_v_pid_conf = {
        .Kp = 3,
        .Kd = 0.0f,
        .Ki = 0.0f,
        .MaxOut = 1.5f,
        .DeadBand = 0.0f,
        .Improve = PID_DerivativeFilter | PID_Derivative_On_Measurement,
    };
    PIDInit(&steer_v_pid, &steer_v_pid_conf);

    // // 抗劈叉
    PID_Init_Config_s anti_crash_pid_conf = {
        .Kp = 30,
        .Kd = 2,
        .Ki = 0.0,
        .MaxOut = 60,
        .DeadBand = 0.001f,
        .Improve = PID_DerivativeFilter | PID_ChangingIntegrationRate | PID_Integral_Limit,
        .Derivative_LPF_RC = 0.01,
    };
    PIDInit(&anti_crash_pid, &anti_crash_pid_conf);

    lidar_info = LidarInit(&huart1);

    // 状态初始化
    l_side.target_len = r_side.target_len = 0.16;
    SpeedEstimationInit();
    DWT_GetDeltaT(&balance_dwt_cnt);

    PID_Init_Config_s charge_buffer_config = {
        .Kp = 3.0f,
        .Ki = 0,
        .Kd = 0,
        .IntegralLimit = 10,
        .MaxOut = 90,
        .DeadBand = 0.0,
    };
    PIDInit(&charge_buffer_pid, &charge_buffer_config);

    PID_Init_Config_s motion_buffer_config = {
        .Kp = 4.0f,
        .Ki = 0,
        .Kd = 0,
        .IntegralLimit = 10,
        .MaxOut = 120,
        .DeadBand = 0.0,
    };
    PIDInit(&motion_buffer_pid, &motion_buffer_config);

    SuperCap_Init_Config_s cap_conf = {
        .can_config = {
            .can_handle = &hcan1,
            .tx_id = 0x133,  // 超级电容默认接收id
            .rx_id = 0x132,   // 超级电容默认发送id,注意tx和rx在其他人看来是反的
        },
    };
    cap = SuperCapInit(&cap_conf);  // 超级电容初始化
}

static void EnableAllMotor() /* 打开所有电机 */
{
    for (uint8_t i = 0; i < JOINT_CNT; i++)  // 打开关节电机
        DMMotorEnable(joint[i]);
    for (uint8_t i = 0; i < DRIVEN_CNT; i++)  // 打开驱动电机
        DJIMotorEnable(driven[i]);
}

static void StopAllMotor() /* 关闭所有电机 */
{
    for (uint8_t i = 0; i < JOINT_CNT; i++)  // 关闭关节电机
        DMMotorStop(joint[i]);
    for (uint8_t i = 0; i < DRIVEN_CNT; i++)  // 关闭驱动电机
        DJIMotorStop(driven[i]);
}

// 检查关节电机是否离线
static uint8_t JointMotorIsLost()
{
    for (uint8_t i = 0; i < JOINT_CNT; i++)
    {
        if (joint[i]->motor_comm_daemon->temp_count == 0)
            return 1;
    }
    return 0;
}

// 检查驱动轮电机是否离线
static uint8_t DrivenMotorIsLost()
{
    for (uint8_t i = 0; i < DRIVEN_CNT; i++)
    {
        if (driven[i]->daemon->temp_count == 0)
            return 1;
    }
    return 0;
}

// 工作状态设定
static void WokingStateSet()
{
    // --- 正常运行模式的参数设定 ---
    // 腿长设定与限幅
    if (chassis_cmd_recv.leglen_mode == CHASSIS_LEGLEN_NO_CONTROL)
    {
        l_side.target_len += 0.00005f * (float)chassis_cmd_recv.delta_leglen;
        r_side.target_len += 0.00005f * (float)chassis_cmd_recv.delta_leglen;
    }
    else if (chassis_cmd_recv.leglen_mode == CHASSIS_LEGLEN_MID)
    {
        l_side.target_len = r_side.target_len = 0.18;
    }
    else if (chassis_cmd_recv.leglen_mode == CHASSIS_LEGLEN_HIGH)
    {
        l_side.target_len = r_side.target_len = 0.26;
    }

    VAL_LIMIT(l_side.target_len, 0.14, 0.30);
    VAL_LIMIT(r_side.target_len, 0.14, 0.30);

    bool is_rotate = (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE || chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE_REVERSE);

    if (is_rotate)
    {
        float offset_angle_rad = chassis_cmd_recv.offset_angle * DEGREE_2_RAD;

        float v_target = chassis_cmd_recv.vx * arm_sin_f32(offset_angle_rad - PI / 4.0f + Chassis_IMU_data->Gyro[Z] * del_t) -
                         chassis_cmd_recv.vy * arm_cos_f32(offset_angle_rad - PI / 4.0f + Chassis_IMU_data->Gyro[Z] * del_t);

        chassis.target_v = v_target;

        VAL_LIMIT(chassis.target_v, -0.5f, 0.5f);
    }
    else
    {
        float offset_angle_deg = chassis_cmd_recv.offset_angle;
        float velocity_direction = 1.0f;
        float speed_attenuation = 1.0f;

        if (fabsf(offset_angle_deg) > 90.0f)
        {
            if (offset_angle_deg > 0)
            {
                offset_angle_deg -= 180.0f;
            }
            else
            {
                offset_angle_deg += 180.0f;
            }
            velocity_direction = -1.0f;
        }

        float offset_angle_abs_rad = fabsf(offset_angle_deg) * DEGREE_2_RAD;

        float cos_offset_angle = arm_cos_f32(offset_angle_abs_rad);

        if (cos_offset_angle < 0.0f)
            cos_offset_angle = 0.0f;

        speed_attenuation = sqrtf(cos_offset_angle);

        float v_target = chassis_cmd_recv.vx * velocity_direction * speed_attenuation;

        chassis.target_v += sign(v_target - chassis.target_v) * MAX_ACC_REF * del_t;
        VAL_LIMIT(chassis.target_v, -2.5f, 2.5f);

        chassis.target_yaw = chassis.yaw + offset_angle_deg * DEGREE_2_RAD;
    }

    // 急停模式下，将目标重置为当前状态，防止积分累积
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE || chassis_cmd_recv.chassis_mode == CHASSIS_RESET)
    {
        chassis.target_v = 0;
        chassis.dist = chassis.target_dist = 0;
        l_side.target_len = r_side.target_len = 0.18;
        chassis.target_yaw = chassis.yaw;
    }
    float v_ref_max = 0.23f * sqrtf(final_power_limit);

    if (chassis_cmd_recv.motion_mode == CHASSIS_MOTION_FAST)
    {
        VAL_LIMIT(v_ref_max, -2.5f, 2.5f);
    }
    else
    {
        if (chassis_cmd_recv.jump_mode == CHASSIS_JUMP_ON)
            VAL_LIMIT(v_ref_max, -2.0f, 2.0f);
        else
            VAL_LIMIT(v_ref_max, -1.5f, 1.5f);
    }

    VAL_LIMIT(chassis.target_v, -v_ref_max, v_ref_max);
    // TODO 最大dist误差限幅
    // TODO 最大速度误差限幅
}

/**
 * @brief 将电机和imu的数据组装为LinkNPodParam结构体和chassisParam结构体
 *
 * @note HT04电机上电的编码器位置为零(校准过),请看Link2Pod()的note,以及HT04.c中的电机解码部分
 * @note 海泰04电机顺时针旋转为正; LK9025电机逆时针旋转为正,此处皆需要转换为模型中给定的正方向
 *
 */
static void ParamAssemble()
{
    // 机体参数,视为平面刚体
    chassis.yaw = Chassis_IMU_data->YawTotalAngle * DEGREE_2_RAD;
    chassis.wz = Chassis_IMU_data->Gyro[Z];
    chassis.pitch = -Chassis_IMU_data->Pitch * DEGREE_2_RAD;
    chassis.pitch_w = -Chassis_IMU_data->Gyro[Y];
    chassis.roll = Chassis_IMU_data->Roll * DEGREE_2_RAD;
    chassis.roll_w = Chassis_IMU_data->Gyro[X];

    // DM8009电机的角度是逆时针为正,LK9025电机的角度是逆时针为正
    l_side.phi1 = PI + lb->measure.position - LIMIT_LINK_RAD;
    l_side.phi1_w = lb->measure.velocity;
    l_side.T_back_measure = lb->measure.torque;
    l_side.phi4 = lf->measure.position + LIMIT_LINK_RAD;
    l_side.phi4_w = lf->measure.velocity;
    l_side.T_front_measure = lf->measure.torque;
    l_side.w_ecd = -l_driven->measure.speed * RPM_2_RAD_PER_SEC / WHEEL_REDUCTION_RATIO;

    r_side.phi1 = PI - rb->measure.position - LIMIT_LINK_RAD;
    r_side.phi1_w = -rb->measure.velocity;
    r_side.T_back_measure = -rb->measure.torque;
    r_side.phi4 = -rf->measure.position + LIMIT_LINK_RAD;
    r_side.phi4_w = -rf->measure.velocity;
    r_side.T_front_measure = -rf->measure.torque;
    r_side.w_ecd = r_driven->measure.speed * RPM_2_RAD_PER_SEC / WHEEL_REDUCTION_RATIO;
}

static void SynthesizeMotion() /* 腿部控制:抗劈叉; 轮子控制:转向 */
{
    if (chassis_cmd_recv.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)  // 底盘跟随
    {
        float p_ref = PIDCalculate(&steer_p_pid, chassis.yaw, chassis.target_yaw);
        PIDCalculate(&steer_v_pid, chassis.wz, p_ref);
    }
    else if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE)  // 小陀螺
    {
        float v_turn_max = 1.6f * sqrtf(final_power_limit);
        VAL_LIMIT(v_turn_max, 0.0f, 10.0f);
        PIDCalculate(&steer_v_pid, chassis.wz, v_turn_max);
    }
    else if (chassis_cmd_recv.chassis_mode == CHASSIS_ROTATE_REVERSE)
    {
        float v_turn_max = 1.6f * sqrtf(final_power_limit);
        VAL_LIMIT(v_turn_max, 0.0f, 10.0f);
        PIDCalculate(&steer_v_pid, chassis.wz, -v_turn_max);
    }
    if (l_side.fly_flag == 0)
    {
        l_side.T_wheel -= steer_v_pid.Output;
    }

    if (r_side.fly_flag == 0)
    {
        r_side.T_wheel += steer_v_pid.Output;
    }

    // 抗劈叉
    static float swerving_speed_ff, ff_coef = 3;
    swerving_speed_ff = ff_coef * steer_v_pid.Output;  // 用于抗劈叉的前馈
    if (l_side.fly_flag == 1 && r_side.fly_flag == 1)  // 两侧都离地
        swerving_speed_ff = 0;

    PIDCalculate(&anti_crash_pid, l_side.phi0 - r_side.phi0, 0);

    l_side.T_hip += anti_crash_pid.Output - swerving_speed_ff;
    r_side.T_hip -= anti_crash_pid.Output - swerving_speed_ff;
}

static bool jump_state = false;
static bool last_jump_state = false;
static bool has_auto_jumped = false;
static float jump_start_time = 0;
static float jump_now_time = 0;
static float k_jump_time = 0.25f;  // 跳跃时间
static float k_retract_time = 0.20f;
static void Jump()
{
    if (l_side.leg_len > 0.18f && r_side.leg_len > 0.18f && last_jump_state == false)
    {
        l_side.target_len = r_side.target_len = 0.16f;
        last_jump_state = false;
        return;
    }
    else
    {
        if (jump_state == true && last_jump_state == false)
        {
            jump_start_time = DWT_GetTimeline_s();
        }

        jump_now_time = DWT_GetTimeline_s();

        if (fabsf(jump_now_time - jump_start_time) <= k_jump_time)
        {
            l_side.F_leg = r_side.F_leg = 220.0f;
        }

        jump_now_time = DWT_GetTimeline_s();

        if ((jump_now_time - jump_start_time - k_jump_time) <= k_retract_time && (jump_now_time - jump_start_time) > k_jump_time)
        {
            l_side.F_leg = r_side.F_leg = -120.0f;
        }

        if ((jump_now_time - jump_start_time - k_jump_time) > k_retract_time)
        {
            jump_state = false;
            l_side.target_len = r_side.target_len = 0.18f;
        }
        last_jump_state = jump_state;
    }
}

static void LegControl() /* 腿长控制和Roll补偿 */
{
    // 重力前馈补偿
    static float gravity_ff = BODY_MASS * K_GRAVITY;  // 可再调
    // 侧向惯性补偿
    float lateral_inertial_comp_l = BODY_MASS * l_side.leg_len / (2 * HALF_WHEEL_DISTANCE) * chassis.wz * chassis.vel;
    float lateral_inertial_comp_r = BODY_MASS * r_side.leg_len / (2 * HALF_WHEEL_DISTANCE) * chassis.wz * chassis.vel;

    float roll_comp = PIDCalculate(&roll_compensation_pid, chassis.roll, chassis_cmd_recv.target_roll);

    if (l_side.fly_flag == 0)
    {
        float l_leg_speed_ref = PIDCalculate(&leg_length_pid_l, l_side.leg_len, l_side.target_len);
        l_side.F_leg = PIDCalculate(&leg_speed_pid_l, l_side.legd, l_leg_speed_ref) + gravity_ff + roll_comp - lateral_inertial_comp_l;
    }
    else
    {
        float l_leg_speed_ref = PIDCalculate(&leg_length_pid_l, l_side.leg_len, l_side.target_len);
        // float l_leg_speed_ref = PIDCalculate(&leg_length_pid_l, l_side.leg_len, 0.28f);
        l_side.F_leg = PIDCalculate(&leg_speed_pid_l, l_side.legd, l_leg_speed_ref) + gravity_ff;
    }

    if (r_side.fly_flag == 0)
    {
        float r_leg_speed_ref = PIDCalculate(&leg_length_pid_r, r_side.leg_len, r_side.target_len);
        r_side.F_leg = PIDCalculate(&leg_speed_pid_r, r_side.legd, r_leg_speed_ref) + gravity_ff - roll_comp + lateral_inertial_comp_r;
    }
    else
    {
        float r_leg_speed_ref = PIDCalculate(&leg_length_pid_r, r_side.leg_len, r_side.target_len);
        // float r_leg_speed_ref = PIDCalculate(&leg_length_pid_r, r_side.leg_len, 0.28f);
        r_side.F_leg = PIDCalculate(&leg_speed_pid_r, r_side.legd, r_leg_speed_ref) + gravity_ff;
    }

    // if (chassis_cmd_recv.jump_mode == CHASSIS_JUMP_ON)
    // {
    //     if (has_auto_jumped == false)
    //     {
    //         if ((0.25f * chassis.vel < lidar_info->distance) && (0.35f * chassis.vel > lidar_info->distance) && (chassis.vel > 1.0f))
    //         {
    //             jump_state = true;
    //             has_auto_jumped = true;
    //         }
    //     }
    // }
    // else
    // {
    //     has_auto_jumped = false;
    // }
    // // if (chassis_cmd_recv.jump_mode == CHASSIS_JUMP_ON)
    // // {
    // //     jump_state = true;
    // // }

    // if (jump_state == true)
    // {
    //     Jump();
    // }

    static bool jump_lock = false;      // 单次按键触发锁：JUMP_OFF 时重置
    static bool is_bursting = false;    // 正在力控收缩阶段
    static float burst_start_time = 0;  // 记录爆发开始时间

    // 配置参数
    const float k_burst_force = -120.0f;   // 瞬间收缩力（负值向上）
    const float k_burst_duration = 0.20f;  // 爆发持续时间
    const float k_hold_len = 0.15f;        // 爆发后的保持长度

    if (chassis_cmd_recv.jump_mode == CHASSIS_JUMP_ON)
    {
        // --- 1. 触发判断（仅在没上锁且没在执行时，检测角度） ---
        if (!jump_lock && !is_bursting)
        {
            float l_leg_speed_ref = PIDCalculate(&leg_length_pid_l, l_side.leg_len, 0.33f);
            l_side.F_leg = PIDCalculate(&leg_speed_pid_l, l_side.legd, l_leg_speed_ref) + gravity_ff + roll_comp - lateral_inertial_comp_l;

            float r_leg_speed_ref = PIDCalculate(&leg_length_pid_r, r_side.leg_len, 0.33f);
            r_side.F_leg = PIDCalculate(&leg_speed_pid_r, r_side.legd, r_leg_speed_ref) + gravity_ff - roll_comp + lateral_inertial_comp_r;

            if (fabsf(l_side.theta * RAD_2_DEGREE) > 23.0f || fabsf(r_side.theta * RAD_2_DEGREE) > 23.0f)
            {
                is_bursting = true;
                burst_start_time = DWT_GetTimeline_s();
            }
        }

        // --- 2. 动作执行（状态机逻辑） ---
        if (is_bursting)
        {
            float elapsed = DWT_GetTimeline_s() - burst_start_time;

            if (elapsed <= k_burst_duration)
            {
                // 【瞬间爆发阶段】：直接控力，实现极速收腿
                l_side.F_leg = r_side.F_leg = k_burst_force;
                l_side.T_wheel = r_side.T_wheel = 0;
            }
            else
            {
                // 【爆发结束】：切换回位置控制，保持缩腿高度
                l_side.target_len = r_side.target_len = k_hold_len;

                // 关键点：完成动作后上锁，并结束爆发状态
                is_bursting = false;
                jump_lock = true;
            }
        }
    }
    else  // 即 chassis_cmd_recv.jump_mode == CHASSIS_JUMP_OFF
    {
        // 只有人松开了按键，才解锁，允许下一次上台阶
        jump_lock = false;
        is_bursting = false;
    }
}

static void MotorOutputSet() /* 设定运动模态的输出 */
{
    // Case A: 急停 (Zero Force)
    // 虽然前面算了一堆 VMC 力矩，但在这里被拦截，直接给电机发 Stop
    if (chassis_cmd_recv.chassis_mode == CHASSIS_ZERO_FORCE)
    {
        StopAllMotor();
        return;
    }

    // Case B: 复位 (Reset)
    // 复位需要特殊的控制逻辑（比如只动轮子，不动关节），覆盖掉 LQR 的结果
    if (chassis_cmd_recv.chassis_mode == CHASSIS_RESET)
    {
        EnableAllMotor();
        // 关节复位逻辑
        // 这里示例沿用你之前的驱动轮复位逻辑
        // 关节电机如果需要保持软力矩或者归零，需在此处设置
        return;
    }

    // Case C: 正常运行
    // 使用 VMCProject 计算出的结果
    EnableAllMotor();

    // SuperCapSend(cap, power_limit);

    DMMotorSetRef(lf, l_side.T_front);
    DMMotorSetRef(lb, l_side.T_back);
    DMMotorSetRef(rf, -r_side.T_front);
    DMMotorSetRef(rb, -r_side.T_back);
    DJIMotorSetRef(l_driven, -l_side.T_wheel * 2598.15f);
    DJIMotorSetRef(r_driven, r_side.T_wheel * 2598.15f);
    // LKMotorSetRef(l_driven, 195.3125f * l_side.T_wheel);
    // LKMotorSetRef(r_driven, 195.3125f * -r_side.T_wheel);
    // LKMotorSetRef(l_driven, 195.3125f * final_T_L);
    // LKMotorSetRef(r_driven, 195.3125f * -final_T_R);
}

// 裁判系统,双板通信,电容功率控制等
static void CommNPower()
{
    static uint8_t chassis_feedback_cnt = 0;
    chassis_feedback_data.bullet_speed = referee_data->ShootData.bullet_speed;
    chassis_feedback_data.bullet_cnt = referee_data->bullet_cnt;
    chassis_feedback_data.rest_heat = referee_data->GameRobotState.shooter_barrel_heat_limit - referee_data->PowerHeatData.shooter_17mm_1_barrel_heat;
    chassis_feedback_data.shooter_cooling_value = referee_data->GameRobotState.shooter_barrel_cooling_value;
    ui_data.ui_mode = chassis_cmd_recv.ui_mode;
    ui_data.yaw_offset_angle = chassis_cmd_recv.offset_angle;
    ui_data.chassis_mode = chassis_cmd_recv.chassis_mode;
    ui_data.chassis_leglen_mode = chassis_cmd_recv.leglen_mode;
    ui_data.chassis_motion_mode = chassis_cmd_recv.motion_mode;
    ui_data.friction_mode = chassis_cmd_recv.friction_mode;
    ui_data.vision_mode = chassis_cmd_recv.vision_mode;
    ui_data.loader_mode = chassis_cmd_recv.loader_mode;
    ui_data.cap_remain_power = (cap->cap_msg.vol - 8.0f) / (23.5f - 8.0f);
    ui_data.cap_remain_power = float_constrain(ui_data.cap_remain_power, 0.0f, 1.0f);
    ui_data.robot_hp = referee_data->GameRobotState.current_HP;
    ui_data.remain_bullet = 500 - referee_data->bullet_cnt;
    memcpy(ui_data.coord, l_side.coord, sizeof(l_side.coord));
    if (chassis_feedback_cnt++ % 2 == 0)
    {
        CANCommSend(cmd_can_comm, (void *)&chassis_feedback_data);
        chassis_feedback_cnt = 0;
    }
}

void BalanceTask()
{
    del_t = DWT_GetDeltaT(&balance_dwt_cnt);

    BuzzerOn();
    // 切换遥控器控制or云台板控制
    chassis_cmd_recv = *(Chassis_Ctrl_Cmd_s *)CANCommGet(cmd_can_comm);

    if (JointMotorIsLost() || DrivenMotorIsLost())
        chassis_cmd_recv.chassis_mode = CHASSIS_ZERO_FORCE;  // 皆离线,急停

    // 充电功率
    power_limit = referee_data->GameRobotState.chassis_power_limit - PIDCalculate(&charge_buffer_pid, referee_data->PowerHeatData.buffer_energy, 40);

    if (cap->cap_msg.vol > 11.0f)
    {
        final_power_limit = referee_data->GameRobotState.chassis_power_limit + 120;
    }
    else
    {
        final_power_limit = referee_data->GameRobotState.chassis_power_limit;
    }

    if (chassis_cmd_recv.motion_mode == CHASSIS_MOTION_SLOW)
        final_power_limit -= PIDCalculate(&motion_buffer_pid, referee_data->PowerHeatData.buffer_energy, 60);

    // 设置目标参数和工作模式
    WokingStateSet();
    // 参数组装
    ParamAssemble();
    // 将五连杆映射成单杆
    Link2Leg(&l_side, &chassis, del_t);
    Link2Leg(&r_side, &chassis, del_t);
    // 根据
    NormalForceSolve(&l_side, Chassis_IMU_data);
    NormalForceSolve(&r_side, Chassis_IMU_data);
    // 通过卡尔曼滤波估计机体速度
    SpeedCalc(&l_side, &r_side, &chassis, Chassis_IMU_data, del_t);
    // 根据单杆计算处的角度和杆长,计算反馈增益
    CalcLQR_MPC_Fusion(&l_side, &chassis);
    CalcLQR_MPC_Fusion(&r_side, &chassis);
    // 转向和抗劈叉
    SynthesizeMotion();
    // 腿长控制,保持机体水平
    LegControl();
    // VMC映射成关节输出
    VMCProject(&l_side);
    VMCProject(&r_side);

    // 运动模态,电机输出映射和限幅
    MotorOutputSet();

    // 裁判系统,双板通信,电容功率控制等
    CommNPower();
}
