/* This file is intentionally user-editable. Add custom PID/control here. */
#include "wheel_leg_user.h"
void WheelLeg_UserInitialize(P_wheel_leg_T *p,DW_wheel_leg_T *dw)
{
    (void)p;(void)dw;
    /* Bind application PID instances after wheel_leg_initialize(). */
}
void WheelLeg_UserBeforeStep(ExtU_wheel_leg_T *u,const P_wheel_leg_T *p,DW_wheel_leg_T *dw)
{
    (void)u;(void)p;(void)dw;
    /* Example: filter or modify u->velocity_command_m_s before the model runs. */
}
void WheelLeg_UserAfterControl(const ExtU_wheel_leg_T *u,const B_wheel_leg_T *b,
                               const P_wheel_leg_T *p,DW_wheel_leg_T *dw,ExtY_wheel_leg_T *y)
{
    (void)u;(void)b;(void)p;(void)dw;(void)y;
    /* Example custom PID:
     * real32_T correction=wheel_leg_call_pid(0u,u->roll_rad,0.0f);
     * y->joint_torque_nm[0]+=correction; y->joint_torque_nm[2]-=correction;
     * Add final saturation here after changing motor commands.
     */
}
