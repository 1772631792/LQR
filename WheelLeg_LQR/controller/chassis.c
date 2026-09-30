#include "chassis.h"
#include <math.h>
#include <string.h>

typedef struct
{
    float kp, ki, kd, integral;
} PID;
static struct
{
    float gains[3][4], ts, wheel_limit, joint_limit;
    PID length[2], roll, yaw;
    int ready;
} mcu;
static float clip(float v, float lo, float hi) { return fminf(hi, fmaxf(lo, v)); }
static float pid(PID *p, float error, float rate, float limit)
{
    float candidate = clip(p->integral + p->ki * error * mcu.ts, -limit, limit);
    float raw = p->kp * error + candidate - p->kd * rate;
    if (fabsf(raw) <= limit || raw * error < 0)
    {
        p->integral = candidate;
    }
    return clip(p->kp * error + p->integral - p->kd * rate, -limit, limit);
}

/* Symmetric outward-knee five-bar IK and analytic J^T*[0,-support].
 * Applied axial force is derated when either joint reaches its torque limit. */
static int vmc(float height, float *force, float *ta, float *te)
{
    const float upper = 0.13f, lower = 0.20f, half = 0.06f;
    float distance = hypotf(half, height), along = (upper * upper - lower * lower + distance * distance) / (2 * distance);
    float rad = upper * upper - along * along, bx, bz, cbx, cbz, dx, dz, det, jza, jze, ratio;
    if (rad <= 1e-10f)
    {
        return -2;
    }
    bx = -half + along * half / distance - sqrtf(rad) * height / distance;
    bz = -along * height / distance - sqrtf(rad) * half / distance;
    dx = -bx;
    dz = bz;
    cbx = -bx;
    cbz = -height - bz;
    det = cbx * (-height - dz) - (-dx) * cbz;
    if (fabsf(det) < 1e-8f)
    {
        return -2;
    }
    jza = -(-dx) * (cbx * (-bz) + cbz * (bx + half)) / det;
    jze = cbx * ((-dx) * (-dz) + (-height - dz) * (dx - half)) / det;
    *ta = -jza * (*force);
    *te = -jze * (*force);
    ratio = fmaxf(1.0f, fmaxf(fabsf(*ta), fabsf(*te)) / mcu.joint_limit);
    *force /= ratio;
    *ta /= ratio;
    *te /= ratio;
    return 0;
}

int Chassis_Init(const double *gains, const double *tuning, double ts)
{
    int i, j;
    memset(&mcu, 0, sizeof(mcu));
    if (!gains || !tuning || !isfinite(ts) || ts < 0.0001 || ts > 0.01)
    {
        return -1;
    }
    for (i = 0; i < 12; ++i)
    {
        if (!isfinite(gains[i]))
        {
            return -1;
        }
    }
    for (i = 0; i < 11; ++i)
    {
        if (!isfinite(tuning[i]) || tuning[i] < 0)
        {
            return -1;
        }
    }
    if (tuning[9] <= 0 || tuning[10] <= 0)
    {
        return -1;
    }
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 4; ++j)
        {
            mcu.gains[i][j] = (float)gains[i * 4 + j];
        }
    for (i = 0; i < 2; ++i)
    {
        mcu.length[i].kp = (float)tuning[0];
        mcu.length[i].ki = (float)tuning[1];
        mcu.length[i].kd = (float)tuning[2];
    }
    mcu.roll.kp = (float)tuning[3];
    mcu.roll.ki = (float)tuning[4];
    mcu.roll.kd = (float)tuning[5];
    mcu.yaw.kp = (float)tuning[6];
    mcu.yaw.ki = (float)tuning[7];
    mcu.yaw.kd = (float)tuning[8];
    mcu.ts = (float)ts;
    mcu.wheel_limit = (float)tuning[9];
    mcu.joint_limit = (float)tuning[10];
    mcu.ready = 1;
    return 0;
}

int Chassis_Update(const double *s, const double *ref, double *out)
{
    float h, alpha, drive = 0, roll_torque, yaw_torque, fl, fr, requested_fl, requested_fr, tl, tr, ta, te;
    int i, node;
    if (!s || !ref || !out || !mcu.ready)
    {
        return -1;
    }
    for (i = 0; i < 10; ++i)
    {
        out[i] = 0;
        if (!isfinite(s[i]))
        {
            return -1;
        }
    }
    for (i = 0; i < 4; ++i)
    {
        if (!isfinite(ref[i]))
        {
            return -1;
        }
    }
    h = (float)s[4];
    node = (h <= 0.20f) ? 0 : 1;
    alpha = clip((h - (0.16f + 0.04f * node)) / 0.04f, 0, 1);
    for (i = 0; i < 4; ++i)
    {
        float gain = mcu.gains[node][i] + alpha * (mcu.gains[node + 1][i] - mcu.gains[node][i]);
        drive -= gain * ((float)s[i] - (i == 0 ? (float)ref[0] : 0.0f));
    }
    roll_torque = pid(&mcu.roll, (float)(ref[2] - s[6]), (float)s[7], 4.0f);
    yaw_torque = pid(&mcu.yaw, atan2f(sinf((float)(ref[3] - s[8])), cosf((float)(ref[3] - s[8]))), (float)s[9], 3.0f);
    /* hL=h+track/2*roll, hR=h-track/2*roll (small-roll reduced model). */
    fl = 24.525f * cosf((float)s[2]) + pid(&mcu.length[0], (float)(ref[1] + 0.18 * ref[2] - s[4] - 0.18 * s[6]), (float)(s[5] + 0.18 * s[7]), 65.0f) + roll_torque / 0.36f;
    fr = 24.525f * cosf((float)s[2]) + pid(&mcu.length[1], (float)(ref[1] - 0.18 * ref[2] - s[4] + 0.18 * s[6]), (float)(s[5] - 0.18 * s[7]), 65.0f) - roll_torque / 0.36f;
    requested_fl = fl;
    requested_fr = fr;
    fl = clip(fl, 0, 100);
    fr = clip(fr, 0, 100);
    if (vmc((float)(s[4] + 0.18 * s[6]), &fl, &ta, &te))
    {
        return -2;
    }
    out[6] = ta;
    out[7] = te;
    if (vmc((float)(s[4] - 0.18 * s[6]), &fr, &ta, &te))
    {
        return -2;
    }
    out[8] = ta;
    out[9] = te;
    /* Track actual actuator authority after support/joint saturation. */
    mcu.length[0].integral = clip(mcu.length[0].integral + 10 * mcu.ts * (fl - requested_fl), -65, 65);
    mcu.length[1].integral = clip(mcu.length[1].integral + 10 * mcu.ts * (fr - requested_fr), -65, 65);
    /* Axial support and wheel-force equivalent reduced plant; wheel body reaction omitted. */
    tl = clip(0.06f * (drive / 2 - yaw_torque / 0.36f), -mcu.wheel_limit, mcu.wheel_limit);
    tr = clip(0.06f * (drive / 2 + yaw_torque / 0.36f), -mcu.wheel_limit, mcu.wheel_limit);
    out[0] = (tl + tr) / 0.06f;
    out[1] = fl;
    out[2] = fr;
    out[3] = (tr - tl) * 0.18f / 0.06f;
    out[4] = tl;
    out[5] = tr;
    mcu.yaw.integral = clip(mcu.yaw.integral + 10 * mcu.ts * ((float)out[3] - yaw_torque), -3, 3);
    return 0;
}
