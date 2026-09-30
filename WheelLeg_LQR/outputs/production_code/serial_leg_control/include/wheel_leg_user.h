#ifndef WHEEL_LEG_USER_H
#define WHEEL_LEG_USER_H
#include "wheel_leg.h"
void WheelLeg_UserInitialize(P_wheel_leg_T *parameters,DW_wheel_leg_T *states);
void WheelLeg_UserBeforeStep(ExtU_wheel_leg_T *input,const P_wheel_leg_T *parameters,DW_wheel_leg_T *states);
void WheelLeg_UserAfterControl(const ExtU_wheel_leg_T *input,const B_wheel_leg_T *signals,
                               const P_wheel_leg_T *parameters,DW_wheel_leg_T *states,ExtY_wheel_leg_T *output);
#endif
