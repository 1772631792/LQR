#ifndef CHASSIS_H
#define CHASSIS_H
#include "lqr.h"
/* One static virtual MCU per process. Reset before each experiment.
 * state: p,v,pitch,pitch_rate,h,h_rate,roll,roll_rate,yaw,yaw_rate.
 * references: position,height,roll,yaw. SI units throughout.
 * tuning: length kp,ki,kd; roll kp,ki,kd; yaw kp,ki,kd; wheel Nm limit; joint Nm limit.
 * gains: three rows at height 0.16,0.20,0.24 m.
 * output: force_N,left_support_N,right_support_N,yaw_Nm,left_wheel_Nm,right_wheel_Nm,
 *         left_hip_A_Nm,left_hip_E_Nm,right_hip_A_Nm,right_hip_E_Nm.
 */
LQR_API int Chassis_Init(const double *gains, const double *tuning, double ts);
LQR_API int Chassis_Update(const double *state, const double *reference, double *output);
#endif
