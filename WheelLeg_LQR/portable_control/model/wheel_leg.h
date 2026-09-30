#ifndef WHEEL_LEG_H
#define WHEEL_LEG_H
#include "wheel_leg_types.h"
extern ExtU_wheel_leg_T wheel_leg_U;
extern ExtY_wheel_leg_T wheel_leg_Y;
extern B_wheel_leg_T wheel_leg_B;
extern DW_wheel_leg_T wheel_leg_DW;
extern P_wheel_leg_T wheel_leg_P;
void wheel_leg_initialize(void);
void wheel_leg_step(void);
void wheel_leg_terminate(void);
int wheel_leg_bind_pid(uint32_T index,void *instance,WheelLeg_PIDCalculateFn calculate,WheelLeg_PIDResetFn reset);
real32_T wheel_leg_call_pid(uint32_T index,real32_T measure,real32_T reference);
#endif
