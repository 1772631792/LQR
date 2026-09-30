/* Preserved on re-export. Rename/add fields here for application-specific control. */
#ifndef WHEEL_LEG_USER_TYPES_H
#define WHEEL_LEG_USER_TYPES_H
#include "wheel_leg_pid_interface.h"
typedef struct {
    real32_T parameter[32];
} WheelLeg_UserP;
typedef struct {
    real32_T memory[32];
} WheelLeg_UserDW;
#endif
