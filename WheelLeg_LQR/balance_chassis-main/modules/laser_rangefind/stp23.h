#ifndef _STP23_H
#define _STP23_H

#include <stdint.h>
#include "bsp_usart.h"
#include "stm32f4xx_hal_uart.h"

#define LIDAR_HEADER 0x54
#define LIDAR_VERLEN 0x2c
#define POINT_PER_PACK 12
#define LIDAR_FRAME_LEN 47

typedef struct __attribute__((packed)) LidarPoint
{
    uint16_t distance;
    uint8_t intensity;
} LidarPoint_s;

typedef struct __attribute__((packed)) LidarFrame
{
    uint8_t header;
    uint8_t verlen;
    uint16_t temperature;
    uint16_t start_angle;
    LidarPoint_s point[POINT_PER_PACK];
    uint16_t end_angle;
    uint16_t timestamp;
    uint8_t crc8;
} LidarFrame_s;

typedef struct LidarInfo
{
    float distance;
} LidarInfo_s;

LidarInfo_s *LidarInit(UART_HandleTypeDef *lidar_usart_handle);

#endif  // !_STP23_H
