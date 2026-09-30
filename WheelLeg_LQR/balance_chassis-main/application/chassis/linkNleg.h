#include "arm_math.h"
#include "balance.h"
#include "math.h"
#include "user_lib.h"

/* 计算的T_hip和F_Leg映射为关节电机输出 */
void VMCProject(LinkNPodParam *p)
{
    p->T_back = p->j_11 * p->F_leg + p->j_21 * p->T_hip;
    p->T_front = p->j_12 * p->F_leg + p->j_22 * p->T_hip;
}

/**
 * @brief 根据关节角度和角速度,计算单杆长度和角度以及变化率
 *
 * @note 右侧视图
 *  ___x
 * |    1  _____  4
 * |y     /     \
 *      2 \     / 3
 *         \   /
 *          \./
 *           5
 * @param p 5连杆和腿的参数
 */
void Link2Leg(LinkNPodParam *p, ChassisParam *chassis, float del_t)
{
    float xD, yD, xB, yB, BD, A0, B0, xC, yC;
    p->coord[4] = xD = JOINT_DISTANCE + THIGH_LEN * mcos(p->phi4);
    p->coord[5] = yD = THIGH_LEN * msin(p->phi4);
    p->coord[0] = xB = THIGH_LEN * mcos(p->phi1);
    p->coord[1] = yB = THIGH_LEN * msin(p->phi1);

    BD = powf(xD - xB, 2) + powf(yD - yB, 2);
    A0 = 2 * CALF_LEN * (xD - xB);
    B0 = 2 * CALF_LEN * (yD - yB);
    p->phi2 = 2 * atan2f(B0 + Sqrt(powf(A0, 2) + powf(B0, 2) - powf(BD, 2)), A0 + BD);
    p->coord[2] = xC = xB + CALF_LEN * mcos(p->phi2);
    p->coord[3] = yC = yB + CALF_LEN * msin(p->phi2);
    p->phi3 = atan2f(yC - yD, xC - xD); // 稍后用于计算VMC

    // 虚拟腿解算
    float x_rel = xC - JOINT_DISTANCE / 2.0f; // 轮轴相对于机体中心的X坐标
    p->phi0 = atan2f(yC, x_rel);
    p->leg_len = Sqrt(powf(x_rel, 2) + powf(yC, 2));
    p->theta = p->phi0 - 0.5f * PI - chassis->pitch;
    // p->height = p->leg_len * mcos(p->theta); // 如果需要高度可以取消注释

    // 2.1 计算五连杆雅可比相关的三角项
    // 速度闭链方程: l2*w2*sin(3-2) = l1*w1*sin(1-3) + l4*w4*sin(3-4)
    float sin_32 = msin(p->phi3 - p->phi2);
    float sin_12 = msin(p->phi1 - p->phi2);
    float sin_34 = msin(p->phi3 - p->phi4);

    float sin_03 = msin(p->phi0 - p->phi3);
    float cos_03 = mcos(p->phi0 - p->phi3);
    float sin_02 = msin(p->phi0 - p->phi2);
    float cos_02 = mcos(p->phi0 - p->phi2);

    float inv_sin_32 = 1.0f / sin_32;
    p->j_11 = (THIGH_LEN * sin_03 * sin_12) * inv_sin_32;
    p->j_12 = (THIGH_LEN * sin_02 * sin_34) * inv_sin_32;

    float inv_L0 = 1.0f / p->leg_len;
    p->j_21 = (THIGH_LEN * cos_03 * sin_12) * inv_sin_32 * inv_L0;
    p->j_22 = (THIGH_LEN * cos_02 * sin_34) * inv_sin_32 * inv_L0;

    float proj_left = THIGH_LEN * sin_12;
    float proj_right = THIGH_LEN * sin_34;

    p->t_11 = -cos_02 / proj_left;
    p->t_12 = cos_03 / proj_right;
    p->t_21 = (p->leg_len * sin_02) / proj_left;
    p->t_22 = -(p->leg_len * sin_03) / proj_right;

    // 腿长变化率 legd = J11 * w1 + J12 * w4
    p->legd = p->j_11 * p->phi1_w + p->j_12 * p->phi4_w;
    p->legd_dot = 0.05f * ((p->legd - p->last_legd) / del_t) + 0.95f * p->legd_dot;
    p->last_legd = p->legd;

    // 虚拟腿摆动角速度 phi0_w = J21 * w1 + J22 * w4
    p->phi0_w = p->j_21 * p->phi1_w + p->j_22 * p->phi4_w;

    // 倒立摆角度变化率 theta_w
    p->theta_w = p->phi0_w - chassis->pitch_w;
    p->theta_w_dot = 0.2f * ((p->theta_w - p->last_theta_w) / del_t) + 0.8f * p->theta_w_dot;
    p->last_theta_w = p->theta_w;

    // 连杆2角速度 phi2_w 用于轮速修正
    float sin_13 = msin(p->phi1 - p->phi3);
    p->phi2_w = (THIGH_LEN * p->phi1_w * sin_13 + THIGH_LEN * p->phi4_w * sin_34) / (CALF_LEN * sin_32);
}
