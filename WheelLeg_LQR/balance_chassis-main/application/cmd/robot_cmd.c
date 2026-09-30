// app
#include "robot_cmd.h"
#include "robot_def.h"
// module
#include "bmi088.h"
#include "can_comm.h"
#include "dji_motor.h"
#include "general_def.h"
#include "ins_task.h"
#include "master_process.h"
#include "message_center.h"
#include "remote_control.h"
#include "stm32f4xx_hal_def.h"
#include "user_lib.h"
// bsp
#include "bsp_dwt.h"
#include "bsp_log.h"
#include "string.h"

static Publisher_t *chassis_cmd_pub;              // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub;            // 底盘反馈信息订阅者
static Chassis_Ctrl_Cmd_s chassis_cmd_send;       // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data;  // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等

static RC_ctrl_t *rc_data;                  // 遥控器数据,初始化时返回
static VisionToGimbal_s *vision_recv_data;  // 视觉接收数据指针,初始化时返回据

static Publisher_t *gimbal_cmd_pub;             // 云台控制消息发布者
static Subscriber_t *gimbal_feed_sub;           // 云台反馈信息订阅者
static Gimbal_Ctrl_Cmd_s gimbal_cmd_send;       // 传递给云台的控制信息
static Gimbal_Upload_Data_s gimbal_fetch_data;  // 从云台获取的反馈信息

static Publisher_t *shoot_cmd_pub;            // 发射控制消息发布者
static Subscriber_t *shoot_feed_sub;          // 发射反馈信息订阅者
static Shoot_Ctrl_Cmd_s shoot_cmd_send;       // 传递给发射的控制信息
static Shoot_Upload_Data_s shoot_fetch_data;  // 从发射获取的反馈信息

static Robot_Status_e robot_state;  // 机器人整体工作状态

static CANCommInstance *cmd_can_comm;  // 底盘CAN通信实例

static chassis_direction_e chassis_direction;  // 底盘方向

void RobotCMDInit()
{
    rc_data = RemoteControlInit(&huart3);    // 修改为对应串口,注意如果是自研板dbus协议串口需选用添加了反相器的那个
    vision_recv_data = VisionInit(&huart1);  // 视觉通信串口

    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Ctrl_Cmd_s));
    gimbal_feed_sub = SubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
    shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_Ctrl_Cmd_s));
    shoot_feed_sub = SubRegister("shoot_feed", sizeof(Shoot_Upload_Data_s));

    CANComm_Init_Config_s comm_conf = {
        .can_config = {
            .can_handle = &hcan2,
            .tx_id = 0x312,
            .rx_id = 0x311,
        },
        .recv_data_len = sizeof(Chassis_Upload_Data_s),
        .send_data_len = sizeof(Chassis_Ctrl_Cmd_s),
    };
    cmd_can_comm = CANCommInit(&comm_conf);

    gimbal_cmd_send.pitch = 0;
    gimbal_cmd_send.yaw = 0;
    gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
    chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
    shoot_cmd_send.shoot_mode = SHOOT_OFF;
    shoot_cmd_send.load_mode = LOAD_STOP;
    shoot_cmd_send.friction_mode = FRICTION_OFF;
    chassis_direction = CHASSIS_ALIGN;
    chassis_cmd_send.vision_mode = VISION_AUTOAIM;

    robot_state = ROBOT_READY;  // 启动时机器人进入工作模式,后续加入所有应用初始化完成之后再进入
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 *        单圈绝对角度的范围是0~360,说明文档中有图示
 *
 */
static void CalcOffsetAngle()
{
    // @todo:相差一整圈时会出问题，待修复
    // 别名angle提高可读性,不然太长了不好看,虽然基本不会动这个函数
    uint16_t yaw_chassis_align_ecd = 0;
    if (chassis_direction == CHASSIS_ALIGN)
    {
        yaw_chassis_align_ecd = YAW_CHASSIS_ALIGN_ECD;
    }
    else if (chassis_direction == CHASSIS_SIDLE)
    {
        yaw_chassis_align_ecd = YAW_CHASSIS_SIDE_ECD;
    }

    static float angle, yaw_align_angle;
    yaw_align_angle = yaw_chassis_align_ecd * ECD_ANGLE_COEF_DJI;  // 从底盘获取的yaw电机对齐角度
    angle = gimbal_fetch_data.yaw_motor_single_round_angle;        // 从云台获取的当前yaw电机单圈角度

    if (yaw_chassis_align_ecd > 4096)  // 如果大于180度
    {
        if (angle > yaw_align_angle)
            chassis_cmd_send.offset_angle = angle - yaw_align_angle;
        else if (angle <= yaw_align_angle && angle >= yaw_align_angle - 180.0f)
            chassis_cmd_send.offset_angle = angle - yaw_align_angle;
        else
            chassis_cmd_send.offset_angle = angle - yaw_align_angle + 360.0f;
    }
    else
    {  // 小于180度
        if (angle > yaw_align_angle && angle <= 180.0f + yaw_align_angle)
            chassis_cmd_send.offset_angle = angle - yaw_align_angle;
        else if (angle > 180.0f + yaw_align_angle)
            chassis_cmd_send.offset_angle = angle - yaw_align_angle - 360.0f;
        else
            chassis_cmd_send.offset_angle = angle - yaw_align_angle;
    }
}

float MapTargetToTotal(float normalized_target, float current_total)
{
    float current_normalized = fmodf(current_total, 360.0f);
    // 处理 fmodf 的负数余数
    if (current_normalized > 180.0f)
        current_normalized -= 360.0f;
    if (current_normalized < -180.0f)
        current_normalized += 360.0f;

    float error = normalized_target - current_normalized;

    // 寻找最短路径跨越边界
    if (error > 180.0f)
        error -= 360.0f;
    else if (error < -180.0f)
        error += 360.0f;

    return current_total + error;
}

static void ShootHeatControl(float shooter_barrel_cooling_value, float rest_heat, loader_mode_e load_mode)
{
    float a = shooter_barrel_cooling_value;
    float m = rest_heat;
    float d = 10;

    static uint16_t allow_shoot_time = 0;
    static uint16_t now_shoot_time = 0;

    if (load_mode == LOAD_STOP || (m >= 150.0f && now_shoot_time == allow_shoot_time))
    {
        now_shoot_time = 0;
    }

    if (now_shoot_time == 0)
    {
        allow_shoot_time = (m + 2 * a) * 10;

        LIMIT_MIN_MAX(allow_shoot_time, 20, 3000);

        if (m < 100)
        {
            shoot_cmd_send.shoot_rate = (10 * m - a - 3 * d) / (d * (allow_shoot_time / 100.0f)) + a / d - 1.0f;
        }
        else
        {
            shoot_cmd_send.shoot_rate = (10 * m - a - 5 * d) / (d * (allow_shoot_time / 100.0f)) + a / d - 1.0f;
        }
    }
    else if (0 < now_shoot_time && now_shoot_time < allow_shoot_time) {}
    else
    {
        shoot_cmd_send.shoot_rate = (a / d) - 1.0f;
    }

    if (now_shoot_time < allow_shoot_time)
    {
        now_shoot_time++;
    }

    LIMIT_MIN_MAX(shoot_cmd_send.shoot_rate, 0, 14.0f);
}

/**
 * @brief 控制输入为遥控器(调试时)的模式和控制量设置
 *
 */
static void RemoteControlSet()
{
    memset(&rc_data->key_count, 0, sizeof(rc_data->key_count));
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;

    // 云台参数,确定云台控制数据
    if (switch_is_up(rc_data->rc.switch_left) && (vision_recv_data->mode != VISION_MODE_NO_CONTROL))
    {
        gimbal_cmd_send.yaw = MapTargetToTotal(vision_recv_data->yaw, gimbal_fetch_data.gimbal_imu_data.YawTotalAngle);
        // gimbal_cmd_send.yaw = vision_recv_data->yaw;
        gimbal_cmd_send.pitch = vision_recv_data->pitch;
    }
    else
    {
        gimbal_cmd_send.yaw -= 0.001f * (float)rc_data[TEMP].rc.rocker_r_;
        gimbal_cmd_send.pitch -= 0.0006f * (float)rc_data[TEMP].rc.rocker_r1;
    }
    chassis_cmd_send.delta_leglen = 0.0f;
    // 摇杆控制的软件限位
    gimbal_cmd_send.pitch = float_constrain(gimbal_cmd_send.pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);

    // if (switch_is_down(rc_data[TEMP].rc.switch_left)) // 左侧开关状态为[下],视觉模式
    // {
    //     gimbal_cmd_send.yaw = (vision_recv_data->yaw == 0 ? gimbal_cmd_send.yaw : vision_recv_data->yaw);
    //     gimbal_cmd_send.pitch = (vision_recv_data->pitch == 0 ? gimbal_cmd_send.pitch : vision_recv_data->pitch);
    // }

    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.target_roll = rc_data[TEMP].rc.rocker_l_ * 0.023 * DEGREE_2_RAD;
    }
    else
    {
        chassis_cmd_send.target_roll = 0;
    }

    chassis_cmd_send.vx = 0.0038f * ((float)rc_data[TEMP].rc.rocker_l1);
    chassis_cmd_send.vy = 0.0038f * ((float)rc_data[TEMP].rc.rocker_l_);

    chassis_cmd_send.motion_mode = CHASSIS_MOTION_FAST;

    // 右侧开关状态为[下],底盘旋转
    if (switch_is_up(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
        chassis_cmd_send.vy = 0;
    }
    else if (switch_is_mid(rc_data[TEMP].rc.switch_right))
    {
        chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
    }
    else if (switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        if (switch_is_up(rc_data[TEMP].rc.switch_left))
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE_REVERSE;
        else
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
    }

    // 摩擦轮控制,拨轮向上打为负,向下为正
    if (switch_is_up(rc_data[TEMP].rc.switch_left))
    {
        shoot_cmd_send.friction_mode = FRICTION_ON;
        if (rc_data[TEMP].rc.dial < -500)
        {
            // i
            
                shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            // }
            // else
            // {
            //     if (vision_recv_data->mode == VISION_MODE_CONTROL_AND_FIRE)
            //         shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            //     else
            //         shoot_cmd_send.load_mode = LOAD_STOP;
            // }
            // if (vision_recv_data->mode == VISION_MODE_CONTROL_AND_FIRE)
            //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
            // else
            //     shoot_cmd_send.load_mode = LOAD_STOP;
        }
        else
        {
            shoot_cmd_send.load_mode = LOAD_STOP;
        }

        // if (rc_data[TEMP].rc.dial < -500 && vision_recv_data->mode == VISION_MODE_CONTROL_AND_FIRE)
        //     shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        // else
        //     shoot_cmd_send.load_mode = LOAD_STOP;

        if (rc_data[TEMP].rc.dial > 500)  // 向下超过500,跳跃
            chassis_cmd_send.jump_mode = CHASSIS_JUMP_ON;
        else
            chassis_cmd_send.jump_mode = CHASSIS_JUMP_OFF;
    }
    else
    {
        shoot_cmd_send.friction_mode = FRICTION_OFF;
    }

    chassis_cmd_send.leglen_mode = CHASSIS_LEGLEN_NO_CONTROL;
    // 左侧开关状态为[中]，腿长控制
    if (switch_is_mid(rc_data[TEMP].rc.switch_left))
    {
        chassis_cmd_send.delta_leglen = -(abs(rc_data[TEMP].rc.dial) > 100 ? (int8_t)(rc_data[TEMP].rc.dial / 30) : 0);
    }

    ShootHeatControl(chassis_fetch_data.shooter_cooling_value, chassis_fetch_data.rest_heat, shoot_cmd_send.load_mode);
}

/**
 * @brief 输入为键鼠时模式和控制量设置
 *
 */
static void MouseKeySet()
{
    gimbal_cmd_send.gimbal_mode = GIMBAL_GYRO_MODE;  // 云台默认为陀螺仪模式
    shoot_cmd_send.shoot_mode = SHOOT_ON;

    if (rc_data[TEMP].mouse.press_r && (vision_recv_data->mode != VISION_MODE_NO_CONTROL))  // 鼠标右键视觉模式
    {
        gimbal_cmd_send.yaw = MapTargetToTotal(vision_recv_data->yaw, gimbal_fetch_data.gimbal_imu_data.YawTotalAngle);
        gimbal_cmd_send.pitch = vision_recv_data->pitch;
    }
    else
    {
        gimbal_cmd_send.yaw -= (float)(rc_data[TEMP].mouse.x) / 660.0f * 10.0f;  // 系数待测
        gimbal_cmd_send.pitch -= (float)(rc_data[TEMP].mouse.y) / 660.0f * 10.0f;
    }

    static uint8_t last_key_press = 0;
    if (rc_data[TEMP].key[KEY_PRESS].x)
    {
        if (last_key_press == 0)
        {
            gimbal_cmd_send.yaw -= 180.0f;
        }
        last_key_press = 1;
    }
    else
    {
        last_key_press = 0;
    }

    gimbal_cmd_send.pitch = float_constrain(gimbal_cmd_send.pitch, PITCH_MIN_ANGLE, PITCH_MAX_ANGLE);  // 限制pitch角度

    if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)  // 底盘速度输入 默认无前进轮速
    {
        chassis_cmd_send.vx =
            BALANCE_MAX_SPEED * (float)(rc_data[TEMP].key[KEY_PRESS].w - rc_data[TEMP].key[KEY_PRESS].s - rc_data[TEMP].key[KEY_PRESS].a + rc_data[TEMP].key[KEY_PRESS].d);
    }
    else if (chassis_cmd_send.chassis_mode == CHASSIS_ROTATE)
    {
        chassis_cmd_send.vx = BALANCE_MAX_SPEED * (float)(rc_data[TEMP].key[KEY_PRESS].w - rc_data[TEMP].key[KEY_PRESS].s);
        chassis_cmd_send.vy = BALANCE_MAX_SPEED * (float)(rc_data[TEMP].key[KEY_PRESS].a - rc_data[TEMP].key[KEY_PRESS].d);
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_C] % 2)  // C 高低腿长
    {
        case 0:
            chassis_cmd_send.leglen_mode = CHASSIS_LEGLEN_MID;
            break;
        default:
            chassis_cmd_send.leglen_mode = CHASSIS_LEGLEN_HIGH;
            break;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_G] % 2)  // G 运动速度
    {
        case 0:
            chassis_cmd_send.motion_mode = CHASSIS_MOTION_SLOW;
            break;
        default:
            chassis_cmd_send.motion_mode = CHASSIS_MOTION_FAST;
            break;
    }

    switch (rc_data[TEMP].key[KEY_PRESS].shift)
    {
        case 0:
            chassis_cmd_send.jump_mode = CHASSIS_JUMP_OFF;
            break;
        default:
            chassis_cmd_send.jump_mode = CHASSIS_JUMP_ON;
            break;
    }

    if (chassis_cmd_send.chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
    {
        // wsad控制底盘方向 但会有额外速度输入 不使用
        if (rc_data[TEMP].key[KEY_PRESS].w || rc_data[TEMP].key[KEY_PRESS].s)
        {
            chassis_direction = CHASSIS_ALIGN;
        }
        else if (rc_data[TEMP].key[KEY_PRESS].a || rc_data[TEMP].key[KEY_PRESS].d)
        {
            chassis_direction = CHASSIS_SIDLE;
        }
    }
    else if (chassis_cmd_send.chassis_mode == CHASSIS_ROTATE)
    {
        chassis_direction = CHASSIS_ALIGN;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_F] % 2)  // F 摩擦轮
    {
        case 0:
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            break;
        default:
            shoot_cmd_send.friction_mode = FRICTION_ON;
            break;
    }

    chassis_cmd_send.vision_mode = VISION_AUTOAIM;

    switch (rc_data[TEMP].key[KEY_PRESS].v)
    {
        case 0:
            break;
        default:
            chassis_cmd_send.vision_mode = VISION_SMALL_BUFF;
            break;
    }

    switch (rc_data[TEMP].key[KEY_PRESS].b)
    {
        case 0:
            break;
        default:
            chassis_cmd_send.vision_mode = VISION_BIG_BUFF;
            break;
    }

    if (shoot_cmd_send.friction_mode == FRICTION_ON)
    {
        if (chassis_cmd_send.vision_mode == VISION_AUTOAIM)
            shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
        else
            shoot_cmd_send.load_mode = LOAD_1_BULLET;

        if (rc_data[TEMP].mouse.press_r)
        {
            if (vision_recv_data->mode == VISION_MODE_CONTROL_AND_FIRE)
            {
                if (chassis_cmd_send.vision_mode == VISION_AUTOAIM)
                    shoot_cmd_send.load_mode = LOAD_BURSTFIRE;
                else
                    shoot_cmd_send.load_mode = LOAD_1_BULLET;
            }
            else
                shoot_cmd_send.load_mode = LOAD_STOP;
        }

        if (rc_data[TEMP].mouse.press_l)  // 鼠标左键发射
        {
        }
        else
            shoot_cmd_send.load_mode = LOAD_STOP;
    }
    else
        shoot_cmd_send.load_mode = LOAD_STOP;

    switch (rc_data[TEMP].key[KEY_PRESS].z)  // Z键刷新UI
    {
        case 0:
            chassis_cmd_send.ui_mode = UI_KEEP;
            break;
        default:
            chassis_cmd_send.ui_mode = UI_REFRESH;
            break;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS][Key_R] % 2)  // F 摩擦轮
    {
        case 0:
            chassis_cmd_send.chassis_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
            break;
        default:
            chassis_cmd_send.chassis_mode = CHASSIS_ROTATE;
            break;
    }

    switch (rc_data[TEMP].key[KEY_PRESS].e)
    {
        case 0:
            break;
        default:
            shoot_cmd_send.load_mode = LOAD_REVERSE;
            break;
    }

    ShootHeatControl(chassis_fetch_data.shooter_cooling_value, chassis_fetch_data.rest_heat, shoot_cmd_send.load_mode);
}

/**
 * @brief  紧急停止,包括遥控器左上侧拨轮打满/重要模块离线/双板通信失效等
 *         停止的阈值'300'待修改成合适的值,或改为开关控制.
 *
 * @todo   后续修改为遥控器离线则电机停止(关闭遥控器急停),通过给遥控器模块添加daemon实现
 *
 */
static void EmergencyHandler()
{
    uint8_t remote_has_connected = ((rc_data[TEMP].rc.switch_left != 0) && (rc_data[TEMP].rc.switch_right != 0));
    if (remote_has_connected)  // 拨杆值为0时,即遥控器从上电后未连接过
    {
        robot_state = ROBOT_READY;
        shoot_cmd_send.shoot_mode = SHOOT_ON;
    }

    // 将急停写在最后,确保能正常触发急停
    if (switch_is_down(rc_data[TEMP].rc.switch_left) && switch_is_down(rc_data[TEMP].rc.switch_right))
    {
        robot_state = ROBOT_STOP;
        gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
        chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
        shoot_cmd_send.shoot_mode = SHOOT_OFF;
        shoot_cmd_send.friction_mode = FRICTION_OFF;
        shoot_cmd_send.load_mode = LOAD_STOP;

        gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;
        gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;

        // gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.Yaw;
        // gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
    }

    switch (rc_data[TEMP].key_count[KEY_PRESS_WITH_CTRL][Key_C] % 2)  // ctrl+c 进入急停
    {
        case 0:
            // 没按保持源状态不变
            // 云台会先自动恢复
            break;

        default:
            robot_state = ROBOT_STOP;
            gimbal_cmd_send.gimbal_mode = GIMBAL_ZERO_FORCE;
            chassis_cmd_send.chassis_mode = CHASSIS_ZERO_FORCE;
            shoot_cmd_send.shoot_mode = SHOOT_OFF;
            shoot_cmd_send.friction_mode = FRICTION_OFF;
            shoot_cmd_send.load_mode = LOAD_STOP;

            gimbal_cmd_send.yaw = gimbal_fetch_data.gimbal_imu_data.YawTotalAngle;  // 急停时设定值保持与实际值同步，避免恢复时疯转
            gimbal_cmd_send.pitch = gimbal_fetch_data.gimbal_imu_data.Pitch;
            break;
    }
}
static uint8_t can_comm_cnt = 0;
/* 机器人核心控制任务,200Hz频率运行(必须高于视觉发送频率) */
void RobotCMDTask()
{
    chassis_fetch_data = *(Chassis_Upload_Data_s *)CANCommGet(cmd_can_comm);
    SubGetMessage(shoot_feed_sub, &shoot_fetch_data);
    SubGetMessage(gimbal_feed_sub, &gimbal_fetch_data);

    // 根据gimbal的反馈值计算云台和底盘正方向的夹角,不需要传参,通过static私有变量完成
    CalcOffsetAngle();
    // 根据遥控器左侧开关,确定当前使用的控制模式为遥控器调试还是键鼠
    if (switch_is_up(rc_data[TEMP].rc.switch_left) || switch_is_mid(rc_data[TEMP].rc.switch_left))  // 遥控器左侧开关状态为[下],遥控器控制
        RemoteControlSet();
    if (switch_is_down(rc_data[TEMP].rc.switch_left))  // 遥控器左侧开关状态为[上],键盘控制
        MouseKeySet();

    EmergencyHandler();  // 处理模块离线和遥控器急停等紧急情况

    shoot_cmd_send.bullet_cnt = chassis_fetch_data.bullet_cnt;
    shoot_cmd_send.bullet_speed = chassis_fetch_data.bullet_speed;

    chassis_cmd_send.direction = chassis_direction;
    // UI数据
    chassis_cmd_send.friction_mode = shoot_cmd_send.friction_mode;
    chassis_cmd_send.loader_mode = shoot_cmd_send.load_mode;

    can_comm_cnt++;
    if (can_comm_cnt % 2 == 0)
    {
        CANCommSend(cmd_can_comm, (void *)&chassis_cmd_send);
        can_comm_cnt = 0;
    }

    UpdateVisionMode(chassis_cmd_send.vision_mode, chassis_fetch_data.bullet_speed, chassis_fetch_data.bullet_cnt);

    PubPushMessage(shoot_cmd_pub, (void *)&shoot_cmd_send);
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_cmd_send);
}
