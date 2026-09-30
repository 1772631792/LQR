#ifndef MASTER_PROCESS_H
#define MASTER_PROCESS_H

#include "bsp_usart.h"

#define VISION_SEND_SIZE 256

#pragma pack(1)

typedef struct GimbalToVision
{
    uint8_t head[2];
    uint8_t mode; // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4];   // wxyz顺序
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count; // 子弹累计发送次数
    uint16_t crc16;
} GimbalToVision_s;

typedef struct VisionToGimbal
{
    uint8_t head[2];
    uint8_t mode; // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
} VisionToGimbal_s;

typedef enum GimbalMode
{
    GIMBAL_MODE_IDLE = 0,       // 空闲
    GIMBAL_MODE_AUTO_AIM = 1,   // 自瞄
    GIMBAL_MODE_SMALL_BUFF = 2, // 小符
    GIMBAL_MODE_BIG_BUFF = 3    // 大符
} GimbalMode_e;

typedef enum VisionMode
{
    VISION_MODE_NO_CONTROL = 0,       // 不控制
    VISION_MODE_CONTROL = 1,          // 控制云台但不开火
    VISION_MODE_CONTROL_AND_FIRE = 2, // 控制云台且开火
} VisionMode_e;

#pragma pack()

/**
 * @brief 调用此函数初始化和视觉的串口通信
 *
 * @param handle 用于和视觉通信的串口handle(C板上一般为USART1,丝印为USART2,4pin)
 */
VisionToGimbal_s *VisionInit(UART_HandleTypeDef *_handle);

/**
 * @brief 发送视觉数据
 *
 */
void VisionSend();

void ToVisionUpdate(uint8_t mode, float *q, float yaw, float yaw_vel,
                    float pitch, float pitch_vel, float bullet_speed, uint16_t bullet_count);

void UpdateVisionMode(uint8_t mode, float bullet_speed, uint16_t bullet_count);

void UpdateVisionAttitude(float *q, float yaw, float yaw_vel, float pitch, float pitch_vel);

#endif // !MASTER_PROCESS_H