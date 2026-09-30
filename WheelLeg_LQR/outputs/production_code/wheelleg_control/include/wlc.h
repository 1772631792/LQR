/* Portable wheel-leg controller API.
 * No HAL/RTOS/heap dependency. All units are SI and Wlc_Step runs once per tick.
 * Portions of the kinematics/control equations derive from balance_chassis-main
 * (MIT, Copyright 2022 NeoZng); see LICENSES/NeoZng-MIT.txt in an export. */
#ifndef WLC_H
#define WLC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define WLC_API_VERSION 1u
#define WLC_LEG_COUNT 2u
#define WLC_JOINT_COUNT 4u
#define WLC_MOTOR_COUNT 6u
#define WLC_PID_PORT_COUNT 8u

enum {
    WLC_PID_LEFT_LENGTH=0u,WLC_PID_LEFT_SPEED=1u,
    WLC_PID_RIGHT_LENGTH=2u,WLC_PID_RIGHT_SPEED=3u,
    WLC_PID_ROLL=4u,WLC_PID_YAW_ANGLE=5u,
    WLC_PID_YAW_RATE=6u,WLC_PID_ANTI_SPLIT=7u
};

typedef float (*WlcPidCalculateFn)(void *instance,float measure,float reference);
typedef void (*WlcPidResetFn)(void *instance);
typedef struct { void *instance;WlcPidCalculateFn calculate;WlcPidResetFn reset; } WlcPidPort;

enum {
    WLC_MODE_JUMP = 1u << 0,
    WLC_MODE_ZERO_FORCE = 1u << 1
};

enum {
    WLC_STATUS_OK = 0u,
    WLC_STATUS_LEFT_AIRBORNE = 1u << 0,
    WLC_STATUS_RIGHT_AIRBORNE = 1u << 1,
    WLC_STATUS_JUMP_BURST = 1u << 2,
    WLC_STATUS_JUMP_LOCKED = 1u << 3,
    WLC_STATUS_INPUT_INVALID = 1u << 8,
    WLC_STATUS_KINEMATICS_INVALID = 1u << 9
};

typedef struct {
    float pitch_rad;
    float pitch_rate_rad_s;
    float roll_rad;
    float roll_rate_rad_s;
    float yaw_rad;
    float yaw_rate_rad_s;
    float distance_m;
    float velocity_m_s;
    /* Order: left back, left front, right back, right front. */
    float joint_angle_rad[WLC_JOINT_COUNT];
    float joint_rate_rad_s[WLC_JOINT_COUNT];
    float normal_force_n[WLC_LEG_COUNT];
    float time_s;
} WlcInput;

typedef struct {
    float velocity_m_s;
    float yaw_rate_rad_s;
    float leg_length_m;
    uint32_t mode_flags;
} WlcCommand;

typedef struct {
    float wheel_torque_nm[WLC_LEG_COUNT];
    /* Order: left back, left front, right back, right front. */
    float joint_torque_nm[WLC_JOINT_COUNT];
    uint32_t status_flags;
} WlcOutput;

typedef struct {
    float leg_length_m[WLC_LEG_COUNT];
    float virtual_leg_angle_rad[WLC_LEG_COUNT];
    float leg_rate_m_s[WLC_LEG_COUNT];
    float virtual_leg_rate_rad_s[WLC_LEG_COUNT];
    float support_force_n[WLC_LEG_COUNT];
    float virtual_hip_torque_nm[WLC_LEG_COUNT];
    float target_velocity_m_s;
    float target_distance_m;
    float target_yaw_rad;
} WlcDiagnostics;

typedef struct {
    float sample_time_s;
    float thigh_length_m;
    float calf_length_m;
    float joint_distance_m;
    float wheel_distance_m;
    float body_mass_kg;
    float max_acceleration_m_s2;
    float airborne_force_n;
    float wheel_torque_limit_nm;
    float joint_torque_limit_nm;
    float min_leg_length_m;
    float max_leg_length_m;
    float lqr_coefficients[12][4];
    float mpc_coefficients[12][4];
} WlcConfig;

/* Public so applications can allocate it statically. Treat fields as private. */
typedef struct {
    WlcConfig config;
    float leg[2][24];
    float speed_integral[2];
    float target_velocity;
    float target_distance;
    float target_yaw;
    float burst_start;
    WlcPidPort pid[WLC_PID_PORT_COUNT];
    uint32_t jump_locked;
    uint32_t burst;
    uint32_t initialized;
} WlcContext;

void Wlc_DefaultConfig(WlcConfig *config);
int Wlc_Init(WlcContext *context, const WlcConfig *config);
void Wlc_Reset(WlcContext *context, const WlcInput *initial_input);
int Wlc_Step(WlcContext *context, const WlcInput *input,
             const WlcCommand *command, WlcOutput *output,
             WlcDiagnostics *diagnostics);
uint32_t Wlc_ApiVersion(void);
int Wlc_BindPid(WlcContext *context,uint32_t index,void *instance,WlcPidCalculateFn calculate,WlcPidResetFn reset);
float Wlc_CallPid(WlcContext *context,uint32_t index,float measure,float reference);

#ifdef __cplusplus
}
#endif
#endif
