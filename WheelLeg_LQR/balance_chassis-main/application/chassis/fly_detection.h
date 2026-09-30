#include "arm_math.h"
#include "balance.h"
#include "ins_task.h"
#include "math.h"
#include "user_lib.h"

// 驱动轮支持力解算
void NormalForceSolve(LinkNPodParam *p, INS_t *imu)
{
    float cos_theta = mcos(p->theta);
    float sin_theta = msin(p->theta);

    float acc_body_z = imu->MotionAccel_n[Z];

    float term1 = -(p->legd_dot) * cos_theta;
    float term2 = 2.0f * (p->legd) * (p->theta_w) * sin_theta;
    float term3 = (p->leg_len) * (p->theta_w_dot) * sin_theta;
    float term4 = (p->leg_len) * (p->theta_w) * (p->theta_w) * cos_theta;

    p->zw_ddot = acc_body_z + term1 + term2 + term3 + term4;
    // 驱动轮支持力解算
    p->F_leg_measure = p->t_11 * p->T_back_measure + p->t_12 * p->T_front_measure;
    p->T_hip_measure = p->t_21 * p->T_back_measure + p->t_22 * p->T_front_measure;

    float P;
    P = p->F_leg_measure * cos_theta + p->T_hip_measure * sin_theta / p->leg_len;
    p->normal_force = P + WHEEL_MASS * K_GRAVITY + WHEEL_MASS * p->zw_ddot;
    // p->normal_force = P + WHEEL_MASS * K_GRAVITY;

    // // 离地检测
    if (p->normal_force < 20.0f)
        p->fly_flag = 1;
    else
        p->fly_flag = 0;
    // p->fly_flag = 1;

}