#include "wheel_leg.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
typedef struct { int calls; } ExternalPID;
static float external_pid_calculate(void *instance,float measure,float reference) {
    ExternalPID *pid=(ExternalPID *)instance;pid->calls++;return reference-measure;
}
static void external_pid_reset(void *instance) { ((ExternalPID *)instance)->calls=0; }
int main(void) {
    int tick,motor;ExternalPID external_pid;wheel_leg_initialize();
    assert(wheel_leg_bind_pid(0u,&external_pid,external_pid_calculate,external_pid_reset)==0);
    assert(wheel_leg_call_pid(0u,2.0f,5.0f)==3.0f&&external_pid.calls==1);
    wheel_leg_U.joint_angle_rad[0]=wheel_leg_U.joint_angle_rad[2]=2.8857736f;
    wheel_leg_U.joint_angle_rad[1]=wheel_leg_U.joint_angle_rad[3]=0.2558225f;
    wheel_leg_U.normal_force_n[0]=wheel_leg_U.normal_force_n[1]=80.0f;
    wheel_leg_U.leg_length_command_m=.18f;
    for(tick=0;tick<3000;++tick) {
        wheel_leg_U.time_s=tick*wheel_leg_P.core.sample_time_s;wheel_leg_U.velocity_command_m_s=tick>500?.4f:0.0f;
        wheel_leg_step();
        for(motor=0;motor<2;++motor)assert(isfinite(wheel_leg_Y.wheel_torque_nm[motor])&&fabsf(wheel_leg_Y.wheel_torque_nm[motor])<=wheel_leg_P.core.wheel_torque_limit_nm);
        for(motor=0;motor<4;++motor)assert(isfinite(wheel_leg_Y.joint_torque_nm[motor])&&fabsf(wheel_leg_Y.joint_torque_nm[motor])<=wheel_leg_P.core.joint_torque_limit_nm);
    }
    assert(external_pid.calls>3000); /* Port 0 is called by the left leg length loop. */
    /* The same generated core also executes two-link serial-leg kinematics. */
    wheel_leg_P.core.leg_topology=WLC_LEG_SERIAL;
    wheel_leg_U.joint_angle_rad[0]=wheel_leg_U.joint_angle_rad[2]=-1.7148122f;
    wheel_leg_U.joint_angle_rad[1]=wheel_leg_U.joint_angle_rad[3]=2.3051922f;
    wheel_leg_step();
    assert(isfinite(wheel_leg_B.leg_length_m[0]));
    wheel_leg_U.zero_force_command=1u;wheel_leg_step();
    for(motor=0;motor<2;++motor)assert(wheel_leg_Y.wheel_torque_nm[motor]==0.0f);
    for(motor=0;motor<4;++motor)assert(wheel_leg_Y.joint_torque_nm[motor]==0.0f);
    printf("PASS wheel_leg lifecycle + external PID interface; DW=%u bytes\n",(unsigned)sizeof(DW_wheel_leg_T));wheel_leg_terminate();
    return 0;
}
