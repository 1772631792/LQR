#ifndef RM_REFEREE_H
#define RM_REFEREE_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "bsp_usart.h"
#include "master_process.h"
#include "referee_protocol.h"
#include "robot_def.h"
#include "usart.h"

extern uint8_t UI_Seq;

#pragma pack(1)
typedef struct
{
    uint8_t Robot_Color;         // 机器人颜色
    uint16_t Robot_ID;           // 本机器人ID
    uint16_t Cilent_ID;          // 本机器人对应的客户端ID
    uint16_t Receiver_Robot_ID;  // 机器人车间通信时接收者的ID，必须和本机器人同颜色
} referee_id_t;

// 此结构体包含裁判系统接收数据以及UI绘制与机器人车间通信的相关信息
typedef struct
{
    referee_id_t referee_id;

    xFrameHeader FrameHeader;  // 接收到的帧头信息
    uint16_t CmdID;
    ext_game_state_t GameState;                           // 0x0001
    ext_game_result_t GameResult;                         // 0x0002
    ext_game_robot_HP_t GameRobotHP;                      // 0x0003
    ext_event_data_t EventData;                           // 0x0101
    ext_referee_warning_t RefereeWarning;                 // 0x0104
    ext_dart_info_t DartInfo;                             // 0x0105
    ext_game_robot_state_t GameRobotState;                // 0x0201
    ext_power_heat_data_t PowerHeatData;                  // 0x0202
    ext_game_robot_pos_t GameRobotPos;                    // 0x0203
    ext_buff_musk_t BuffMusk;                             // 0x0204
    ext_robot_hurt_t RobotHurt;                           // 0x0206
    ext_shoot_data_t ShootData;                           // 0x0207
    ext_projectile_allowance_t PojectileAllowance;        // 0X0208
    ext_rfid_status_t RfidStatus;                         // 0X0209
    ext_dart_client_cmd_t DartClientCmd;                  // 0x020A
    ext_ground_robot_position_t ground_robot_p_t;         // 0x020B
    ext_radar_mark_data_t RadarMarkData;                  // 0x020C
    ext_sentry_info_t Sentry_Auto_info;                   // 0x020D
    ext_radar_info_t RadarInfo;                           // 0x020E
    ext_map_command MapCommand;                           // 0x0303
    ext_keyboard_remote_control_t KeyboardRemoteControl;  // 0x0304
    // 自定义交互数据的接收
    // Radar_Data_t ReceiveData;
    uint8_t ReceiveData[128];  // 修改为128字节的数组，确保能够容纳至少127字节
    uint16_t bullet_cnt;

    uint8_t init_flag;

} referee_info_t;

// 模式是否切换标志位，0为未切换，1为切换，static定义默认为0
typedef struct
{
    uint32_t yaw_offset_flag : 1;
    uint32_t chassis_flag : 1;
    uint32_t leglength_flag : 1;
    uint32_t moition_flag : 1;
    uint32_t friction_flag : 1;
    uint32_t vision_flag : 1;
    uint32_t loader_flag : 1;
    uint32_t hp_flag : 1;
    uint32_t bullet_flag : 1;
} Referee_Interactive_Flag_t;

// 此结构体包含UI绘制与机器人车间通信的需要的其他非裁判系统数据
typedef struct
{
    ui_mode_e ui_mode;  // UI状态
    Referee_Interactive_Flag_t Referee_Interactive_Flag;
    // 为UI绘制以及交互数据所用
    float yaw_offset_angle;                     // 底盘姿态指示
    chassis_mode_e chassis_mode;                // 底盘模式
    chassis_leglen_mode_e chassis_leglen_mode;  // 底盘腿长模式
    chassis_motion_mode_e chassis_motion_mode;  // 底盘运动模式
    friction_mode_e friction_mode;              // 摩擦轮模式
    vision_mode_e vision_mode;                  // 视觉模式
    loader_mode_e loader_mode;                  // 拨盘模式
    uint16_t robot_hp;                          // 机器人血量
    uint16_t remain_bullet;                     // 剩余实体弹丸数

    // 上一次的模式，用于flag判断
    float yaw_offset_angle_last;
    chassis_mode_e chassis_last_mode;
    chassis_leglen_mode_e chassis_last_leglen_mode;
    chassis_motion_mode_e chassis_last_motion_mode;
    friction_mode_e friction_last_mode;
    vision_mode_e vision_last_mode;
    loader_mode_e loader_last_mode;
    uint16_t robot_last_hp;
    uint16_t last_remain_bullet;

    float coord[6];          // 五连杆
    float cap_remain_power;  // 超电能量剩余百分比
} Referee_Interactive_info_t;

#pragma pack()

/**
 * @brief 裁判系统通信初始化,该函数会初始化裁判系统串口,开启中断
 *
 * @param referee_usart_handle 串口handle,C板一般用串口6
 * @return referee_info_t* 返回裁判系统反馈的数据,包括热量/血量/状态等
 */
referee_info_t *RefereeInit(UART_HandleTypeDef *referee_usart_handle);

/**
 * @brief UI绘制和交互数的发送接口,由UI绘制任务和多机通信函数调用
 * @note 内部包含了一个实时系统的延时函数,这是因为裁判系统接收CMD数据至高位10Hz
 *
 * @param send 发送数据首地址
 * @param tx_len 发送长度
 */
void RefereeSend(uint8_t *send, uint16_t tx_len);

#endif  // !REFEREE_H
