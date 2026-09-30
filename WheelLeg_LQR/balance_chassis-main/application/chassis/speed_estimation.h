#include "balance.h"
#include "general_def.h"
#include "ins_task.h"
#include "kalman_filter.h"
#include "stdbool.h"
#include "user_lib.h"

static KalmanFilter_t kf;

void SpeedEstimationInit()
{
    // 使用kf同时估计速度和加速度
    Kalman_Filter_Init(&kf, 2, 0, 2);
    float F[4] = {1, 0.001, 0, 1};
    float Q[4] = {VEL_PROCESS_NOISE, 0, 0, ACC_PROCESS_NOISE};
    float R[4] = {VEL_MEASURE_NOISE, 0, 0, ACC_MEASURE_NOISE};
    float P[4] = {100000, 0, 0, 100000};
    float H[4] = {1, 0, 0, 1};
    memcpy(kf.F_data, F, sizeof(F));
    memcpy(kf.Q_data, Q, sizeof(Q));
    memcpy(kf.R_data, R, sizeof(R));
    memcpy(kf.P_data, P, sizeof(P));
    memcpy(kf.H_data, H, sizeof(H));
}

void SpeedCalc(LinkNPodParam *lp, LinkNPodParam *rp, ChassisParam *cp, INS_t *imu, float delta_t)
{
    // 修正轮速
    lp->wheel_w = lp->w_ecd - lp->phi2_w - cp->pitch_w;
    rp->wheel_w = rp->w_ecd - rp->phi2_w - cp->pitch_w;

    // 以轮子为基点,计算机体两侧髋关节处的速度
    lp->body_v = lp->wheel_w * WHEEL_RADIUS + lp->leg_len * lp->theta_w * mcos(lp->theta) + lp->legd * msin(lp->theta);
    rp->body_v = rp->wheel_w * WHEEL_RADIUS + rp->leg_len * rp->theta_w * mcos(rp->theta) + rp->legd * msin(rp->theta);

    cp->vel_m = (lp->body_v + rp->body_v) / 2; // 机体速度(平动)为两侧速度的平均值

    // 使用kf同时估计加速度和速度,滤波更新
    kf.MeasuredVector[0] = cp->vel_m;
    kf.MeasuredVector[1] = imu->MotionAccel_n[X];
    kf.F_data[1] = delta_t;
    Kalman_Filter_Update(&kf);
    cp->vel = kf.xhat_data[0];   // 机体速度
    cp->acc_m = kf.xhat_data[1]; // 机体加速度

    // 速度和位置分离，有速度输入时不进行位置闭环
    if (fabs(cp->target_v) < 0.1f)
    {
        cp->target_dist = 0;
        cp->dist += cp->vel * delta_t;
    }
    else if (lp->fly_flag == 1 && rp->fly_flag == 1)
    {
        cp->target_dist = cp->dist = 0;
    }
    else
    {
        cp->target_dist = cp->dist = 0;
    }
}
