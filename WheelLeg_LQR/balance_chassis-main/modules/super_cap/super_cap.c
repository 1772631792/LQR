#include "super_cap.h"
#include "memory.h"
#include "stdlib.h"

static SuperCapInstance *super_cap_instance = NULL; // 可以由app保存此指针

static void BytesToFloats(uint8_t bytes[8], float *num1, float *num2)
{
    memcpy(num1, bytes, sizeof(float));
    memcpy(num2, bytes + sizeof(float), sizeof(float));
}

static void SuperCapRxCallback(CANInstance *_instance)
{
    float vol, power;
    BytesToFloats(_instance->rx_buff, &vol, &power);
    super_cap_instance->cap_msg.vol = vol;
    super_cap_instance->cap_msg.power = power;
}

SuperCapInstance *SuperCapInit(SuperCap_Init_Config_s *supercap_config)
{
    super_cap_instance = (SuperCapInstance *)malloc(sizeof(SuperCapInstance));
    memset(super_cap_instance, 0, sizeof(SuperCapInstance));

    supercap_config->can_config.can_module_callback = SuperCapRxCallback;
    super_cap_instance->can_ins = CANRegister(&supercap_config->can_config);
    return super_cap_instance;
}

void SuperCapSend(SuperCapInstance *instance, float data)
{
    static int16_t integer = 0;
    static int16_t fractional = 0;

    integer = (int16_t)data;
    fractional = (data - (float)integer) * 1000;
    instance->can_ins->tx_buff[0] = integer >> 8;
    instance->can_ins->tx_buff[1] = integer;
    instance->can_ins->tx_buff[2] = fractional >> 8;
    instance->can_ins->tx_buff[3] = fractional;
    CANTransmit(instance->can_ins, 1);
}

SuperCap_Msg_s SuperCapGet(SuperCapInstance *instance)
{
    return instance->cap_msg;
}