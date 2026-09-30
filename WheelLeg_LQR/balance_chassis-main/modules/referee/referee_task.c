/**
 * @file referee.C
 * @author kidneygood (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-11-18
 *
 * @copyright Copyright (c) 2022
 *
 */
#include "referee_task.h"
#include <stdint.h>
#include "referee_protocol.h"
#include "robot_def.h"
#include "rm_referee.h"
#include "referee_UI.h"
#include "string.h"
#include "cmsis_os.h"
#include "balance.h"

static Referee_Interactive_info_t *Interactive_data;  // UI绘制需要的机器人状态数据
static referee_info_t *referee_recv_info;             // 接收到的裁判系统数据
uint8_t UI_Seq;                                       // 包序号，供整个referee文件使用
// @todo 不应该使用全局变量

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data);  // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data);  // 测试用函数，实现模式自动变化

/**
 * @brief  判断各种ID，选择客户端ID
 * @param  referee_info_t *referee_recv_info
 * @retval none
 * @attention
 */
static void DeterminRobotID()
{
    // id小于7是红色,大于7是蓝色,0为红色，1为蓝色   #define Robot_Red 0    #define Robot_Blue 1
    referee_recv_info->referee_id.Robot_Color = referee_recv_info->GameRobotState.robot_id > 7 ? Robot_Blue : Robot_Red;
    referee_recv_info->referee_id.Robot_ID = referee_recv_info->GameRobotState.robot_id;
    referee_recv_info->referee_id.Cilent_ID = 0x0100 + referee_recv_info->referee_id.Robot_ID;  // 计算客户端ID
    referee_recv_info->referee_id.Receiver_Robot_ID = 0;
}

static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data);
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data);  // 模式切换检测
static void RobotModeTest(Referee_Interactive_info_t *_Interactive_data);  // 测试用函数，实现模式自动变化

referee_info_t *UITaskInit(UART_HandleTypeDef *referee_usart_handle, Referee_Interactive_info_t *UI_data)
{
    referee_recv_info = RefereeInit(referee_usart_handle);  // 初始化裁判系统的串口,并返回裁判系统反馈数据指针
    Interactive_data = UI_data;                             // 获取UI绘制需要的机器人状态数据
    referee_recv_info->init_flag = 1;
    return referee_recv_info;
}

void UITask()
{
    // RobotModeTest(Interactive_data); // 测试用函数，实现模式自动变化,用于检查该任务和裁判系统是否连接正常
    MyUIRefresh(referee_recv_info, Interactive_data);
}

// 射击准线
static Graph_Data_t UI_ShootLine_static[3];

// 能量条
static Graph_Data_t UI_Energy_static[2];
static Graph_Data_t UI_Energy_dynamic;
static String_Data_t UI_Energy_char[2];
// 剩余弹量
static Graph_Data_t UI_RestBullet_static[2];
static Graph_Data_t UI_RestBullet_dynamic;
static String_Data_t UI_RestBullet_char[2];
// Pitch仪表盘
static Graph_Data_t UI_RestHp_static[2];
static Graph_Data_t UI_RestHp_dynamic;
// 底盘姿态显示
static Graph_Data_t UI_YawOffsetAngle_dynamic;
// 摩擦轮状态
static Graph_Data_t UI_Friction_dynamic[2];
// 自瞄状态
static Graph_Data_t UI_AutoAim_dynamic;
// 拨盘状态
static String_Data_t UI_Loader_static;
static String_Data_t UI_Loader_dynamic;
// 腿长状态
static String_Data_t UI_LegLen_static[2];
static Graph_Data_t UI_LegLen_dynamic;
// 运动状态
static String_Data_t UI_Motion_static[2];
static Graph_Data_t UI_Motion_dynamic;
// 弹丸剩余提示
static String_Data_t UI_BulletNotice;

#define Leg_X_Offset 940
#define Leg_Y_Offset 200
#define Leg_Gain 500
static Graph_Data_t Leg_Graph_sta[2];
static Graph_Data_t Leg_Graph_dyn[6];
static uint32_t Processed_coord[6] = { 1605, 455, 1695, 410, 1785, 455 };  // Y坐标取反，放大，平移后的

void MyUIInit()
{
    if (!referee_recv_info->init_flag)
        vTaskDelete(NULL);  // 如果没有初始化裁判系统则直接删除ui任务

    if (Interactive_data->ui_mode == UI_KEEP)
        return;

    while (referee_recv_info->GameRobotState.robot_id == 0)
        osDelay(100);  // 若还未收到裁判系统数据,等待一段时间后再检查

    DeterminRobotID();                                             // 确定ui要发送到的目标客户端
    UIDelete(&referee_recv_info->referee_id, UI_Data_Del_ALL, 0);  // 清空UI

    // 射击准线
    UILineDraw(&UI_ShootLine_static[0], "s22", UI_Graph_ADD, 5, UI_Color_Green, 3, 924, 494, 964, 494);
    UILineDraw(&UI_ShootLine_static[1], "s16", UI_Graph_ADD, 5, UI_Color_Green, 3, 944, 409, 944, 579);
    // UICircleDraw(&UI_ShootLine_static[2], "s90", UI_Graph_ADD, 5, UI_Color_Orange, 3, 952, 508, 30);

    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_ShootLine_static[0], UI_ShootLine_static[1]);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_ShootLine_static[2]);

    // 能量条弧度显示
    UIArcDraw(&UI_Energy_static[0], "en0", UI_Graph_ADD, 7, UI_Color_White, 310, 311, 20, 960, 540, 370, 370);  // F
    UIArcDraw(&UI_Energy_static[1], "en1", UI_Graph_ADD, 7, UI_Color_White, 270, 271, 20, 960, 540, 370, 370);  // E
    UIArcDraw(&UI_Energy_dynamic, "en2", UI_Graph_ADD, 7, UI_Color_Green, 271, 310, 20, 960, 540, 370,
              370);  // 动态能量条

    // 剩余弹量弧度显示
    UIArcDraw(&UI_RestBullet_static[0], "rb0", UI_Graph_ADD, 7, UI_Color_White, 250, 251, 20, 960, 540, 370,
              370);  // 250发
    UIArcDraw(&UI_RestBullet_static[1], "rb1", UI_Graph_ADD, 7, UI_Color_White, 230, 231, 20, 960, 540, 370,
              370);  // 500发
    UIGraphRefresh(&referee_recv_info->referee_id, 5, UI_Energy_static[0], UI_Energy_static[1], UI_Energy_dynamic, UI_RestBullet_static[0], UI_RestBullet_static[1]);

    // 剩余血量
    UIArcDraw(&UI_RestHp_static[0], "rh0", UI_Graph_ADD, 7, UI_Color_White, 50, 51, 40, 960, 540, 370, 370);
    UIArcDraw(&UI_RestHp_static[1], "rh1", UI_Graph_ADD, 7, UI_Color_White, 130, 131, 40, 960, 540, 370, 370);
    UIArcDraw(&UI_RestHp_dynamic, "rh2", UI_Graph_ADD, 7, UI_Color_Green, 51, 130, 40, 960, 540, 370,
              370);  // 动态血量
    UIArcDraw(&UI_YawOffsetAngle_dynamic, "ya0", UI_Graph_ADD, 7, UI_Color_White, 30, 330, 20, 1500, 700, 80,
              80);                                                                                          // 动态底盘姿态
    UILineDraw(&UI_Friction_dynamic[0], "fr0", UI_Graph_ADD, 7, UI_Color_White, 10, 1440, 700, 1480, 700);  // 摩擦轮
    UILineDraw(&UI_Friction_dynamic[1], "fr1", UI_Graph_ADD, 7, UI_Color_White, 10, 1520, 700, 1560, 700);  // 摩擦轮
    UIArcDraw(&UI_AutoAim_dynamic, "au0", UI_Graph_ADD, 7, UI_Color_White, 160, 200, 20, 960, 540, 270,
              270);  // 自瞄状态
    UIGraphRefresh(&referee_recv_info->referee_id, 7, UI_RestHp_static[0], UI_RestHp_static[1], UI_RestHp_dynamic, UI_YawOffsetAngle_dynamic, UI_Friction_dynamic[0],
                   UI_Friction_dynamic[1], UI_AutoAim_dynamic);

    UIArcDraw(&UI_RestBullet_dynamic, "rb3", UI_Graph_ADD, 7, UI_Color_White, 231, 270, 20, 960, 540, 370, 370);
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_RestBullet_dynamic);

    UIRectangleDraw(&UI_LegLen_dynamic, "ll3", UI_Graph_ADD, 7, UI_Color_Green, 10, 685, 870, 795, 800);
    UIRectangleDraw(&UI_Motion_dynamic, "mo3", UI_Graph_ADD, 7, UI_Color_Cyan, 10, 985, 870, 1125, 800);
    UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_LegLen_dynamic, UI_Motion_dynamic);

    // 能量条文字
    UICharDraw(&UI_Energy_char[0], "ec0", UI_Graph_ADD, 7, UI_Color_White, 20, 3, 693, 765, "F");
    UICharDraw(&UI_Energy_char[1], "ec1", UI_Graph_ADD, 7, UI_Color_White, 20, 3, 610, 540, "E");
    // 剩余弹量文字
    UICharDraw(&UI_RestBullet_char[0], "rb4", UI_Graph_ADD, 7, UI_Color_White, 20, 3, 630, 420, "250");
    UICharDraw(&UI_RestBullet_char[1], "rb5", UI_Graph_ADD, 7, UI_Color_White, 20, 3, 692, 315, "500");
    UICharDraw(&UI_Loader_static, "lo0", UI_Graph_ADD, 7, UI_Color_Orange, 40, 3, 130, 850, "Loader:");
    UICharDraw(&UI_Loader_dynamic, "lo1", UI_Graph_ADD, 7, UI_Color_Orange, 40, 3, 410, 850, "STOP");
    UICharDraw(&UI_LegLen_static[0], "ll0", UI_Graph_ADD, 7, UI_Color_Orange, 30, 3, 700, 850, "MID");
    UICharDraw(&UI_LegLen_static[1], "ll1", UI_Graph_ADD, 7, UI_Color_Orange, 30, 3, 830, 850, "HIGH");
    UICharDraw(&UI_Motion_static[0], "mo0", UI_Graph_ADD, 7, UI_Color_Pink, 30, 3, 1000, 850, "SLOW");
    UICharDraw(&UI_Motion_static[1], "mo1", UI_Graph_ADD, 7, UI_Color_Pink, 30, 3, 1150, 850, "FAST");
    UICharDraw(&UI_BulletNotice, "bn", UI_Graph_ADD, 7, UI_Color_White, 50, 3, 740, 680, "NO BULLET");
    UICharRefresh(&referee_recv_info->referee_id, UI_Energy_char[0]);
    UICharRefresh(&referee_recv_info->referee_id, UI_Energy_char[1]);
    UICharRefresh(&referee_recv_info->referee_id, UI_RestBullet_char[0]);
    UICharRefresh(&referee_recv_info->referee_id, UI_RestBullet_char[1]);
    UICharRefresh(&referee_recv_info->referee_id, UI_Loader_static);
    UICharRefresh(&referee_recv_info->referee_id, UI_Loader_dynamic);
    UICharRefresh(&referee_recv_info->referee_id, UI_LegLen_static[0]);
    UICharRefresh(&referee_recv_info->referee_id, UI_LegLen_static[1]);
    UICharRefresh(&referee_recv_info->referee_id, UI_Motion_static[0]);
    UICharRefresh(&referee_recv_info->referee_id, UI_Motion_static[1]);
    UICharRefresh(&referee_recv_info->referee_id, UI_BulletNotice);

    // 绘制车辆状态标志，动态
    // 由于初始化时xxx_last_mode默认为0，所以此处对应UI也应该设为0时对应的UI，防止模式不变的情况下无法置位flag，导致UI无法刷新
    // UICharDraw(&UI_State_dyn[0], "sd0", UI_Graph_ADD, 8, UI_Color_Main, 15, 2, 270, 750, "zeroforce");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[0]);
    // UICharDraw(&UI_State_dyn[1], "sd1", UI_Graph_ADD, 8, UI_Color_Pink, 15, 2, 270, 700, "off");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[1]);
    // UICharDraw(&UI_State_dyn[2], "sd2", UI_Graph_ADD, 8, UI_Color_Orange, 15, 2, 270, 650, "unlock");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[2]);
    // UICharDraw(&UI_State_dyn[4], "sd4", UI_Graph_ADD, 8, UI_Color_Cyan, 15, 2, 270, 600, "stop");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_dyn[4]);

    // // 绘制视觉标识框
    // UIRectangleDraw(&UI_Vision[0], "sd3", UI_Graph_ADD, 9, UI_Color_Green, 2, 650, 700, 1270, 380);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Vision[0]);

    // // 底盘功率显示，静态
    // UICharDraw(&UI_State_sta[5], "ss5", UI_Graph_ADD, 7, UI_Color_Green, 18, 2, 620, 230, "Power:");
    // UICharRefresh(&referee_recv_info->referee_id, UI_State_sta[5]);
    // // 能量条框
    // UIRectangleDraw(&UI_Energy[0], "ss6", UI_Graph_ADD, 7, UI_Color_Green, 2, 720, 140, 1420, 180);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy[0]);

    // // 底盘功率显示,动态
    // UIFloatDraw(&UI_Energy[1], "sd5", UI_Graph_ADD, 8, UI_Color_Green, 18, 2, 2, 750, 230, 24000);
    // // 能量条初始状态
    // UILineDraw(&UI_Energy[2], "sd6", UI_Graph_ADD, 8, UI_Color_Pink, 30, 720, 160, 1020, 160);
    // UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Energy[1], UI_Energy[2]);

    // // 底盘方向--指示云台方向
    // UILineDraw(&Leg_Graph_sta[0], "ws0", UI_Graph_ADD, 9, UI_Color_Purplish_red, 4, 1700, 600, 1700, 705);
    // UIGraphRefresh(&referee_recv_info->referee_id, 1, Leg_Graph_sta[0]);

    // 腿部运动，五连杆
    UILineDraw(&Leg_Graph_dyn[1], "wd1", UI_Graph_ADD, 7, UI_Color_Yellow, 5, 0 + Leg_X_Offset, 0 + Leg_Y_Offset, (uint32_t)(JOINT_DISTANCE * Leg_Gain + Leg_X_Offset),
               0 + Leg_Y_Offset);                                                                                                                              // 水平线
    UILineDraw(&Leg_Graph_dyn[2], "wd2", UI_Graph_ADD, 7, UI_Color_Green, 5, 0 + Leg_X_Offset, 0 + Leg_Y_Offset, Processed_coord[0], Processed_coord[1]);      // 左大腿
    UILineDraw(&Leg_Graph_dyn[3], "wd3", UI_Graph_ADD, 7, UI_Color_Green, 5, Processed_coord[0], Processed_coord[1], Processed_coord[2], Processed_coord[3]);  // 左小腿
    UILineDraw(&Leg_Graph_dyn[4], "wd4", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5, (uint32_t)(JOINT_DISTANCE * Leg_Gain + Leg_X_Offset), 0 + Leg_Y_Offset, Processed_coord[4],
               Processed_coord[5]);                                                                                                                                   // 右大腿
    UILineDraw(&Leg_Graph_dyn[5], "wd5", UI_Graph_ADD, 7, UI_Color_Purplish_red, 5, Processed_coord[2], Processed_coord[3], Processed_coord[4], Processed_coord[5]);  // 右小腿
    UIGraphRefresh(&referee_recv_info->referee_id, 5, Leg_Graph_dyn[1], Leg_Graph_dyn[2], Leg_Graph_dyn[3], Leg_Graph_dyn[4], Leg_Graph_dyn[5]);
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void MyUIRefresh(referee_info_t *referee_recv_info, Referee_Interactive_info_t *_Interactive_data)
{
    UIChangeCheck(_Interactive_data);
    // 底盘姿态指示与底盘模式
    if (_Interactive_data->Referee_Interactive_Flag.yaw_offset_flag == 1 || _Interactive_data->Referee_Interactive_Flag.chassis_flag == 1)
    {
        float a = -_Interactive_data->yaw_offset_angle + 30;
        float b = -_Interactive_data->yaw_offset_angle + 330;
        // 画弧形时，角度不能为负
        while (a < 0)
            a += 360;
        while (b < 0)
            b += 360;

        if (_Interactive_data->chassis_mode == CHASSIS_ZERO_FORCE)
        {
            UIArcDraw(&UI_YawOffsetAngle_dynamic, "ya0", UI_Graph_Change, 7, UI_Color_White, a, b, 20, 1500, 700, 80,
                      80);  // 动态底盘姿态
        }
        if (_Interactive_data->chassis_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
        {
            UIArcDraw(&UI_YawOffsetAngle_dynamic, "ya0", UI_Graph_Change, 7, UI_Color_Green, a, b, 20, 1500, 700, 80,
                      80);  // 动态底盘姿态
        }
        else if (_Interactive_data->chassis_mode == CHASSIS_ROTATE)
        {
            UIArcDraw(&UI_YawOffsetAngle_dynamic, "ya0", UI_Graph_Change, 7, UI_Color_Purplish_red, a, b, 20, 1500, 700, 80, 80);  // 动态底盘姿态
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_YawOffsetAngle_dynamic);
    }
    if (_Interactive_data->Referee_Interactive_Flag.leglength_flag == 1)
    {
        if (_Interactive_data->chassis_leglen_mode == CHASSIS_LEGLEN_MID)
        {
            UIRectangleDraw(&UI_LegLen_dynamic, "ll3", UI_Graph_Change, 7, UI_Color_Green, 10, 685, 870, 795, 800);
        }
        else if (_Interactive_data->chassis_leglen_mode == CHASSIS_LEGLEN_HIGH)
        {
            UIRectangleDraw(&UI_LegLen_dynamic, "ll3", UI_Graph_Change, 7, UI_Color_Green, 10, 815, 870, 955, 800);
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_LegLen_dynamic);
    }
    if (_Interactive_data->Referee_Interactive_Flag.moition_flag == 1)
    {
        if (_Interactive_data->chassis_motion_mode == CHASSIS_MOTION_SLOW)
        {
            UIRectangleDraw(&UI_Motion_dynamic, "mo3", UI_Graph_Change, 7, UI_Color_Cyan, 10, 985, 870, 1125, 800);
        }
        else if (_Interactive_data->chassis_motion_mode == CHASSIS_MOTION_FAST)
        {
            UIRectangleDraw(&UI_Motion_dynamic, "mo3", UI_Graph_Change, 7, UI_Color_Cyan, 10, 1135, 870, 1275, 800);
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Motion_dynamic);
    }
    if (_Interactive_data->Referee_Interactive_Flag.friction_flag == 1)
    {
        if (_Interactive_data->friction_mode == FRICTION_OFF)
        {
            UILineDraw(&UI_Friction_dynamic[0], "fr0", UI_Graph_Change, 7, UI_Color_White, 10, 1440, 700, 1480,
                       700);  // 摩擦轮
            UILineDraw(&UI_Friction_dynamic[1], "fr1", UI_Graph_Change, 7, UI_Color_White, 10, 1520, 700, 1560,
                       700);  // 摩擦轮
        }
        else if (_Interactive_data->friction_mode == FRICTION_ON)
        {
            UILineDraw(&UI_Friction_dynamic[0], "fr0", UI_Graph_Change, 7, UI_Color_Orange, 10, 1440, 700, 1480,
                       700);  // 摩擦轮
            UILineDraw(&UI_Friction_dynamic[1], "fr1", UI_Graph_Change, 7, UI_Color_Orange, 10, 1520, 700, 1560,
                       700);  // 摩擦轮
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 2, UI_Friction_dynamic[0], UI_Friction_dynamic[1]);  // 摩擦轮
    }
    if (_Interactive_data->Referee_Interactive_Flag.vision_flag == 1)
    {
        if (_Interactive_data->vision_mode == VISION_AUTOAIM)
        {
            UIArcDraw(&UI_AutoAim_dynamic, "au0", UI_Graph_Change, 7, UI_Color_Green, 160, 200, 20, 960, 540, 270,
                      270);  // 自瞄状态
        }
        else if (_Interactive_data->vision_mode == VISION_SMALL_BUFF)
        {
            UIArcDraw(&UI_AutoAim_dynamic, "au0", UI_Graph_Change, 7, UI_Color_Pink, 160, 200, 20, 960, 540, 270,
                      270);  // 自瞄状态
        }
        else if (_Interactive_data->vision_mode == VISION_BIG_BUFF)
        {
            UIArcDraw(&UI_AutoAim_dynamic, "au0", UI_Graph_Change, 7, UI_Color_Yellow, 160, 200, 20, 960, 540, 270,
                      270);  // 自瞄状态
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_AutoAim_dynamic);  // 自瞄状态
    }
    if (_Interactive_data->Referee_Interactive_Flag.loader_flag == 1)
    {
        if (_Interactive_data->loader_mode == LOAD_STOP)
        {
            UICharDraw(&UI_Loader_dynamic, "lo1", UI_Graph_Change, 7, UI_Color_Orange, 40, 3, 410, 850, "STOP     ");
        }
        else if (_Interactive_data->loader_mode == LOAD_1_BULLET)
        {
            UICharDraw(&UI_Loader_dynamic, "lo1", UI_Graph_Change, 7, UI_Color_Orange, 40, 3, 410, 850, "ONE      ");
        }
        else if (_Interactive_data->loader_mode == LOAD_BURSTFIRE)
        {
            UICharDraw(&UI_Loader_dynamic, "lo1", UI_Graph_Change, 7, UI_Color_Orange, 40, 3, 410, 850, "BURSTFIRE");
        }
        else if (_Interactive_data->loader_mode == LOAD_REVERSE)
        {
            UICharDraw(&UI_Loader_dynamic, "lo1", UI_Graph_Change, 7, UI_Color_Orange, 40, 3, 410, 850, "REVERSE  ");
        }
        UICharRefresh(&referee_recv_info->referee_id, UI_Loader_dynamic);
    }
    if (Processed_coord[0] != (uint32_t)(_Interactive_data->coord[0] * Leg_Gain + Leg_X_Offset))
    {
        Processed_coord[0] = (uint32_t)(_Interactive_data->coord[0] * Leg_Gain + Leg_X_Offset);
        Processed_coord[1] = (uint32_t)(_Interactive_data->coord[1] * -Leg_Gain + Leg_Y_Offset);
        Processed_coord[2] = (uint32_t)(_Interactive_data->coord[2] * Leg_Gain + Leg_X_Offset);
        Processed_coord[3] = (uint32_t)(_Interactive_data->coord[3] * -Leg_Gain + Leg_Y_Offset);
        Processed_coord[4] = (uint32_t)(_Interactive_data->coord[4] * Leg_Gain + Leg_X_Offset);
        Processed_coord[5] = (uint32_t)(_Interactive_data->coord[5] * -Leg_Gain + Leg_Y_Offset);

        // 腿部运动，五连杆，不需要检测变更，实时显示变化
        UILineDraw(&Leg_Graph_dyn[2], "wd2", UI_Graph_Change, 7, UI_Color_Green, 5, 0 + Leg_X_Offset, 0 + Leg_Y_Offset, Processed_coord[0], Processed_coord[1]);  // 左大腿

        UILineDraw(&Leg_Graph_dyn[3], "wd3", UI_Graph_Change, 7, UI_Color_Green, 5, Processed_coord[0], Processed_coord[1], Processed_coord[2], Processed_coord[3]);  // 左小腿

        UILineDraw(&Leg_Graph_dyn[4], "wd4", UI_Graph_Change, 7, UI_Color_Purplish_red, 5, (uint32_t)(JOINT_DISTANCE * Leg_Gain + Leg_X_Offset), 0 + Leg_Y_Offset,
                   Processed_coord[4],
                   Processed_coord[5]);  // 右大腿

        UILineDraw(&Leg_Graph_dyn[5], "wd5", UI_Graph_Change, 7, UI_Color_Purplish_red, 5, Processed_coord[2], Processed_coord[3], Processed_coord[4],
                   Processed_coord[5]);  // 右小腿
        UIGraphRefresh(&referee_recv_info->referee_id, 5, Leg_Graph_dyn[1], Leg_Graph_dyn[2], Leg_Graph_dyn[3], Leg_Graph_dyn[4], Leg_Graph_dyn[5]);
    }
    // 超电能量实时刷新，不需要检测变更，实时显示变化
    if (_Interactive_data->cap_remain_power > 0.4f)
    {
        UIArcDraw(&UI_Energy_dynamic, "en2", UI_Graph_Change, 7, UI_Color_Green, 271, 272 + (int32_t)(_Interactive_data->cap_remain_power * 38), 20, 960, 540, 370,
                  370);  // 动态能量条
    }
    else
    {
        UIArcDraw(&UI_Energy_dynamic, "en2", UI_Graph_Change, 7, UI_Color_Pink, 271, 272 + (int32_t)(_Interactive_data->cap_remain_power * 38), 20, 960, 540, 370,
                  370);  // 动态能量条
    }
    UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_Energy_dynamic);

    if (_Interactive_data->Referee_Interactive_Flag.hp_flag == 1)
    {
        float rest_hp_percent = (float)referee_recv_info->GameRobotState.current_HP / (float)referee_recv_info->GameRobotState.maximum_HP;
        if (rest_hp_percent > 0.5f)
        {
            UIArcDraw(&UI_RestHp_dynamic, "rh2", UI_Graph_Change, 7, UI_Color_Green, 129 - (129 - 51) * rest_hp_percent, 130, 40, 960, 540, 370,
                      370);  // 动态血量
        }
        else
        {
            UIArcDraw(&UI_RestHp_dynamic, "rh2", UI_Graph_Change, 7, UI_Color_Pink, 129 - (129 - 51) * rest_hp_percent, 130, 40, 960, 540, 370,
                      370);  // 动态血量
        }
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_RestHp_dynamic);
    }

    if (_Interactive_data->Referee_Interactive_Flag.bullet_flag == 1)
    {
        float remain_bullet_percent = (float)_Interactive_data->remain_bullet / 500.0f;

        UIArcDraw(&UI_RestBullet_dynamic, "rb2", UI_Graph_Change, 7, UI_Color_White, 269 - (269 - 230) * remain_bullet_percent, 270, 20, 960, 540, 370, 370);
        UIGraphRefresh(&referee_recv_info->referee_id, 1, UI_RestBullet_dynamic);
    }

    if (referee_recv_info->PojectileAllowance.projectile_allowance_17mm <= 0)
    {
        UICharDraw(&UI_BulletNotice, "bn", UI_Graph_Change, 7, UI_Color_White, 50, 3, 740, 680, "NO  BULLET");
        UICharRefresh(&referee_recv_info->referee_id, UI_BulletNotice);
    }
    else if (referee_recv_info->PojectileAllowance.projectile_allowance_17mm > 0 && referee_recv_info->PojectileAllowance.projectile_allowance_17mm <= 25)
    {
        UICharDraw(&UI_BulletNotice, "bn", UI_Graph_Change, 7, UI_Color_White, 50, 3, 740, 680, "LOW BULLET");
        UICharRefresh(&referee_recv_info->referee_id, UI_BulletNotice);
    }
    else if (referee_recv_info->PojectileAllowance.projectile_allowance_17mm > 25)
    {
        UICharDraw(&UI_BulletNotice, "bn", UI_Graph_Change, 7, UI_Color_White, 50, 3, 740, 680, "          ");
        UICharRefresh(&referee_recv_info->referee_id, UI_BulletNotice);
    }
}

/**
 * @brief  模式切换检测,模式发生切换时，对flag置位
 * @param  Referee_Interactive_info_t *_Interactive_data
 * @retval none
 * @attention
 */
static void UIChangeCheck(Referee_Interactive_info_t *_Interactive_data)
{
    // yaw offset
    if (_Interactive_data->yaw_offset_angle != _Interactive_data->yaw_offset_angle_last)
    {
        _Interactive_data->Referee_Interactive_Flag.yaw_offset_flag = 1;
        _Interactive_data->yaw_offset_angle_last = _Interactive_data->yaw_offset_angle;
    }
    // chassis mode
    if (_Interactive_data->chassis_mode != _Interactive_data->chassis_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.chassis_flag = 1;
        _Interactive_data->chassis_last_mode = _Interactive_data->chassis_mode;
    }
    // chassis leglen mode
    if (_Interactive_data->chassis_leglen_mode != _Interactive_data->chassis_last_leglen_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.leglength_flag = 1;
        _Interactive_data->chassis_last_leglen_mode = _Interactive_data->chassis_leglen_mode;
    }
    // chassis motion mode
    if (_Interactive_data->chassis_motion_mode != _Interactive_data->chassis_last_motion_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.moition_flag = 1;
        _Interactive_data->chassis_last_motion_mode = _Interactive_data->chassis_last_motion_mode;
    }
    // friction mode
    if (_Interactive_data->friction_mode != _Interactive_data->friction_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.friction_flag = 1;
        _Interactive_data->friction_last_mode = _Interactive_data->friction_mode;
    }
    // vision mode
    if (_Interactive_data->vision_mode != _Interactive_data->vision_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.vision_flag = 1;
        _Interactive_data->vision_last_mode = _Interactive_data->vision_mode;
    }
    // loader mode
    if (_Interactive_data->loader_mode != _Interactive_data->loader_last_mode)
    {
        _Interactive_data->Referee_Interactive_Flag.loader_flag = 1;
        _Interactive_data->loader_last_mode = _Interactive_data->loader_mode;
    }
    // remain hp
    if (_Interactive_data->robot_hp != _Interactive_data->robot_last_hp)
    {
        _Interactive_data->Referee_Interactive_Flag.hp_flag = 1;
        _Interactive_data->robot_last_hp = _Interactive_data->robot_hp;
    }
    // remain bullet
    if (_Interactive_data->remain_bullet != _Interactive_data->last_remain_bullet)
    {
        _Interactive_data->Referee_Interactive_Flag.bullet_flag = 1;
        _Interactive_data->last_remain_bullet = _Interactive_data->remain_bullet;
    }
}
