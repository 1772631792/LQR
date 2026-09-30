#include "arm_math.h"
#include "balance.h"
#include "stdint.h"

static float k_lqr[12][4] = {
    { -580.793983, 783.731877, -401.575832, 19.616999 },     // row 0: theta -> wheel
    { -103.178699, 131.034870, -62.460267, 2.731318 },       // row 1: theta_dot -> wheel
    { -155.590732, 183.885801, -81.428101, 6.009046 },       // row 2: x -> wheel
    { -341.501129, 397.130252, -172.488632, 12.079763 },     // row 3: x_dot -> wheel
    { 119.223616, -57.918692, -34.154865, 25.772479 },       // row 4: pitch -> wheel
    { 53.306674, -38.632670, 1.134225, 4.621841 },           // row 5: pitch_dot -> wheel
    { 4690.692116, -4403.513912, 1248.250795, -33.449125 },  // row 6: theta -> hip
    { 855.103215, -779.177375, 211.010110, -4.502903 },      // row 7: theta_dot -> hip
    { 781.538570, -710.924039, 187.640098, -0.134923 },      // row 8: x -> hip
    { 1666.326930, -1506.230392, 393.445801, 0.396554 },     // row 9: x_dot -> hip
    { 367.630338, -548.112480, 296.870504, 22.952638 },      // row 10: pitch -> hip
    { -27.139692, -27.952625, 40.408410, 2.863716 },         // row 11: pitch_dot -> hip
};

static float k_mpc[12][4] = {
    { -27.271530, 70.405511, -33.976441, 1.450837 },     // row 0: theta -> wheel
    { -42.809756, 104.613820, -59.051571, 3.041969 },    // row 1: theta_dot -> wheel
    { -0.012390, 0.020052, -0.010703, 0.001542 },        // row 2: x -> wheel
    { 4.085050, 6.560460, -8.012157, 1.573399 },         // row 3: x_dot -> wheel
    { 2.017577, -1.380650, 0.122355, 0.058932 },         // row 4: pitch -> wheel
    { 32.634487, -27.430700, 5.341886, 0.514025 },       // row 5: pitch_dot -> wheel
    { 398.732249, -404.340647, 113.899198, -2.071685 },  // row 6: theta -> hip
    { 834.653329, -827.303228, 246.558758, -8.398013 },  // row 7: theta_dot -> hip
    { 0.125928, -0.120813, 0.033687, -0.000149 },        // row 8: x -> hip
    { 72.468718, -79.093454, 24.370378, 0.553580 },      // row 9: x_dot -> hip
    { 3.366239, -5.444027, 2.880210, -0.078041 },        // row 10: pitch -> hip
    { -10.879463, -17.219974, 21.690707, -0.201016 },    // row 11: pitch_dot -> hip
};

/**
 * @brief LQR + MPC 融合控制计算
 * @note  同时查两张表，计算基础 LQR 输出和 MPC 修正量，然后加权融合
 */
void CalcLQR_MPC_Fusion(LinkNPodParam *p, ChassisParam *chassis)
{
    // 定义融合权重 (Alpha)
    // 0.0 表示完全不听 MPC 的，1.0 表示完全叠加 MPC
    // 轮子主要由 LQR 驱动，稍微给一点 MPC 修正即可
    const float ALPHA_WHEEL = 0.0f;
    const float ALPHA_HIP = 0.3f;

    // 临时变量
    float T_LQR[2][6] = { 0 };  // LQR 各项分量
    float T_MPC[2][6] = { 0 };  // MPC 各项分量

    float u_lqr[2] = { 0 };  // LQR 总输出 [0]:Wheel, [1]:Hip
    float u_mpc[2] = { 0 };  // MPC 总输出 [0]:Wheel, [1]:Hip

    float l = p->leg_len;
    float lsqr = l * l;
    float lcube = l * l * l;

    // 状态误差向量 (Error State)
    // 注意: 这里与你原代码逻辑保持一致，直接构建每一项的误差
    float state_err[6];
    state_err[0] = -p->theta;                               // Theta (Ref=0)
    state_err[1] = -p->theta_w;                             // Theta_dot
    state_err[2] = (chassis->target_dist - chassis->dist);  // X
    state_err[3] = (chassis->target_v - chassis->vel);      // V
    state_err[4] = -chassis->pitch;                         // Pitch
    state_err[5] = -chassis->pitch_w;                       // Pitch_dot

    // 1. 查表计算增益并乘状态 (并行计算 LQR 和 MPC)
    for (uint8_t i = 0; i < 2; i++)  // i=0: Wheel, i=1: Hip
    {
        uint8_t base_idx = i * 6;  // 索引偏移

        for (uint8_t s = 0; s < 6; s++)  // 遍历6个状态
        {
            uint8_t k_idx = base_idx + s;

            // --- 计算 LQR 分量 ---
            float gain_lqr = k_lqr[k_idx][0] * lcube + k_lqr[k_idx][1] * lsqr + k_lqr[k_idx][2] * l + k_lqr[k_idx][3];
            T_LQR[i][s] = gain_lqr * state_err[s];

            // --- 计算 MPC 分量 ---
            float gain_mpc = k_mpc[k_idx][0] * lcube + k_mpc[k_idx][1] * lsqr + k_mpc[k_idx][2] * l + k_mpc[k_idx][3];
            T_MPC[i][s] = gain_mpc * state_err[s];
        }
    }

    // 2. 离地处理 (Fly Detection)
    if (p->fly_flag)
    {
        // 离地时，轮子输出置零
        for (uint8_t s = 0; s < 6; s++)
        {
            T_LQR[0][s] = 0.0f;
            T_MPC[0][s] = 0.0f;
        }
        // 关节仅保留腿部摆杆
        T_LQR[1][2] = T_LQR[1][3] = T_LQR[1][4] = T_LQR[1][5] = 0;
        T_MPC[1][2] = T_MPC[1][3] = T_MPC[1][4] = T_MPC[1][5] = 0;
    }

    // 累加各项得到总输出 u_lqr, u_mpc
    for (uint8_t i = 0; i < 2; i++)
    {
        for (uint8_t s = 0; s < 6; s++)
        {
            u_lqr[i] += T_LQR[i][s];
            u_mpc[i] += T_MPC[i][s];
        }
    }

    // // 融合输出 (Fusion)
    // // 轮毂力矩: 主要靠 LQR
    p->T_wheel = (1.0f - ALPHA_WHEEL) * u_lqr[0] + ALPHA_WHEEL * u_mpc[0];

    // 关节力矩: 融合 MPC 以抑制震荡
    // 如果 MPC 训练得当，u_mpc 会是一个"刹车"力矩，这里直接相加即可
    // (前提是 Matlab 中 u_mpc 的符号定义与 LQR 一致)
    p->T_hip = (1.0f - ALPHA_HIP) * u_lqr[1] + ALPHA_HIP * u_mpc[1];
}
