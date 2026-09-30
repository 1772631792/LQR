/* Simulink-Coder-style top-level model. Hardware code writes U, calls step, reads Y. */
#include "wheel_leg.h"
#include "wheel_leg_user.h"
#include <string.h>

ExtU_wheel_leg_T wheel_leg_U;
ExtY_wheel_leg_T wheel_leg_Y;
B_wheel_leg_T wheel_leg_B;
DW_wheel_leg_T wheel_leg_DW;
P_wheel_leg_T wheel_leg_P;

int wheel_leg_bind_pid(uint32_T index,void *instance,WheelLeg_PIDCalculateFn calculate,WheelLeg_PIDResetFn reset)
{
    return Wlc_BindPid(&wheel_leg_DW.core,index,instance,calculate,reset);
}

real32_T wheel_leg_call_pid(uint32_T index,real32_T measure,real32_T reference)
{
    return Wlc_CallPid(&wheel_leg_DW.core,index,measure,reference);
}

void wheel_leg_initialize(void)
{
    memset(&wheel_leg_U,0,sizeof(wheel_leg_U));memset(&wheel_leg_Y,0,sizeof(wheel_leg_Y));
    memset(&wheel_leg_B,0,sizeof(wheel_leg_B));memset(&wheel_leg_DW,0,sizeof(wheel_leg_DW));
    memset(&wheel_leg_P,0,sizeof(wheel_leg_P));Wlc_DefaultConfig(&wheel_leg_P.core);
    (void)Wlc_Init(&wheel_leg_DW.core,&wheel_leg_P.core);
    WheelLeg_UserInitialize(&wheel_leg_P,&wheel_leg_DW);
}

void wheel_leg_step(void)
{
    WlcInput input;WlcCommand command;WlcOutput output;WlcDiagnostics diagnostics;
    uint32_T index;
    WheelLeg_UserBeforeStep(&wheel_leg_U,&wheel_leg_P,&wheel_leg_DW);
    memset(&input,0,sizeof(input));memset(&command,0,sizeof(command));
    input.pitch_rad=wheel_leg_U.pitch_rad;input.pitch_rate_rad_s=wheel_leg_U.pitch_rate_rad_s;
    input.roll_rad=wheel_leg_U.roll_rad;input.roll_rate_rad_s=wheel_leg_U.roll_rate_rad_s;
    input.yaw_rad=wheel_leg_U.yaw_rad;input.yaw_rate_rad_s=wheel_leg_U.yaw_rate_rad_s;
    input.distance_m=wheel_leg_U.distance_m;input.velocity_m_s=wheel_leg_U.velocity_m_s;input.time_s=wheel_leg_U.time_s;
    for(index=0;index<4u;++index){input.joint_angle_rad[index]=wheel_leg_U.joint_angle_rad[index];input.joint_rate_rad_s[index]=wheel_leg_U.joint_rate_rad_s[index];}
    for(index=0;index<2u;++index)input.normal_force_n[index]=wheel_leg_U.normal_force_n[index];
    command.velocity_m_s=wheel_leg_U.velocity_command_m_s;command.yaw_rate_rad_s=wheel_leg_U.yaw_rate_command_rad_s;
    command.leg_length_m=wheel_leg_U.leg_length_command_m;
    command.mode_flags=(wheel_leg_U.jump_command?WLC_MODE_JUMP:0u)|(wheel_leg_U.zero_force_command?WLC_MODE_ZERO_FORCE:0u);
    (void)Wlc_Step(&wheel_leg_DW.core,&input,&command,&output,&diagnostics);
    for(index=0;index<2u;++index){wheel_leg_Y.wheel_torque_nm[index]=output.wheel_torque_nm[index];wheel_leg_B.leg_length_m[index]=diagnostics.leg_length_m[index];wheel_leg_B.virtual_leg_angle_rad[index]=diagnostics.virtual_leg_angle_rad[index];wheel_leg_B.leg_rate_m_s[index]=diagnostics.leg_rate_m_s[index];wheel_leg_B.virtual_leg_rate_rad_s[index]=diagnostics.virtual_leg_rate_rad_s[index];wheel_leg_B.support_force_n[index]=diagnostics.support_force_n[index];wheel_leg_B.virtual_hip_torque_nm[index]=diagnostics.virtual_hip_torque_nm[index];}
    for(index=0;index<4u;++index)wheel_leg_Y.joint_torque_nm[index]=output.joint_torque_nm[index];
    wheel_leg_Y.status_flags=output.status_flags;wheel_leg_B.target_velocity_m_s=diagnostics.target_velocity_m_s;
    wheel_leg_B.target_distance_m=diagnostics.target_distance_m;wheel_leg_B.target_yaw_rad=diagnostics.target_yaw_rad;
    WheelLeg_UserAfterControl(&wheel_leg_U,&wheel_leg_B,&wheel_leg_P,&wheel_leg_DW,&wheel_leg_Y);
}

void wheel_leg_terminate(void)
{
    memset(&wheel_leg_Y,0,sizeof(wheel_leg_Y));
}
