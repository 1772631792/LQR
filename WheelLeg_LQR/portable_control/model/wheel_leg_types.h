#ifndef WHEEL_LEG_TYPES_H
#define WHEEL_LEG_TYPES_H
#include "rtwtypes.h"
#include "wheel_leg_user_types.h"
#include "wlc.h"

typedef struct {
    real32_T pitch_rad,pitch_rate_rad_s,roll_rad,roll_rate_rad_s;
    real32_T yaw_rad,yaw_rate_rad_s,distance_m,velocity_m_s;
    real32_T joint_angle_rad[4],joint_rate_rad_s[4],normal_force_n[2],time_s;
    real32_T velocity_command_m_s,yaw_rate_command_rad_s,leg_length_command_m;
    boolean_T jump_command,zero_force_command;
} ExtU_wheel_leg_T;

typedef struct {
    real32_T wheel_torque_nm[2],joint_torque_nm[4];
    uint32_T status_flags;
} ExtY_wheel_leg_T;

/* Block signals: readable intermediate values for custom control additions. */
typedef struct {
    real32_T leg_length_m[2],virtual_leg_angle_rad[2],leg_rate_m_s[2];
    real32_T virtual_leg_rate_rad_s[2],support_force_n[2],virtual_hip_torque_nm[2];
    real32_T target_velocity_m_s,target_distance_m,target_yaw_rad;
} B_wheel_leg_T;

/* User parameters belong here; add more fields without touching the core. */
typedef struct {
    WlcConfig core;
    WheelLeg_UserP user;
} P_wheel_leg_T;

/* Discrete states: generated controller plus user PID/state memory. */
typedef struct {
    WlcContext core;
    WheelLeg_UserDW user;
} DW_wheel_leg_T;
#endif
