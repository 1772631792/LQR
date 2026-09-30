#ifndef __CRC16_H
#define __CRC16_H
#include "main.h"
#include "stdbool.h"

uint16_t CRC16_Calc(const uint8_t *data, uint32_t len);
bool Check_CRC16(const uint8_t *data, uint32_t len);

#endif
