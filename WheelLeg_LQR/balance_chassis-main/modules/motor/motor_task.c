#include "motor_task.h"
#include "LK9025.h"
#include "HT04.h"
#include "dji_motor.h"
#include "step_motor.h"
#include "dji_motor.h"
#include "robot_def.h"
void MotorControlTask()
{
#ifdef GIMBAL_BOARD
    static uint8_t cnt = 0;
    // 设定不同电机的任务频率
    if (cnt % 2 == 0)  // 500hz
    {
        DJIMotorControl();
        cnt = 0;
    }
    cnt++;

#endif  // DEBUG
#ifdef CHASSIS_BOARD
    static uint8_t cnt = 0;
    // 设定不同电机的任务频率
    if (cnt % 2 == 0)  // 500hz
    {
        DJIMotorControl();
        cnt = 0;
    }
    cnt++;

#endif  // DEBUG
}
