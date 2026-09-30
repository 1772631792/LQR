#include "wlc.h"
#include "wlc_generated_params.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#define WLC_PI 3.14159265358979323846f

enum {
    PHI0, PHI1, PHI2, PHI3, PHI4,
    PHI0_RATE, PHI1_RATE, PHI4_RATE,
    LEG_LENGTH, LEG_RATE, LAST_LEG_RATE, LEG_ACCEL,
    THETA, THETA_RATE, LAST_THETA_RATE, THETA_ACCEL,
    J11, J12, J21, J22,
    NORMAL_FORCE, SUPPORT_FORCE, HIP_TORQUE, FLYING
};

static float clipf(float value, float low, float high)
{
    return fminf(high, fmaxf(low, value));
}

static int finite_array(const float *values, size_t count)
{
    size_t index;
    for (index = 0; index < count; ++index) {
        if (!isfinite(values[index])) return 0;
    }
    return 1;
}

static int valid_config(const WlcConfig *c)
{
    if (!c || !finite_array((const float *)c, sizeof(*c) / sizeof(float))) return 0;
    if (c->sample_time_s < 0.0001f || c->sample_time_s > 0.01f) return 0;
    if (c->thigh_length_m <= 0.0f || c->calf_length_m <= 0.0f ||
        c->joint_distance_m <= 0.0f || c->wheel_distance_m <= 0.0f ||
        c->body_mass_kg <= 0.0f || c->max_acceleration_m_s2 <= 0.0f ||
        c->wheel_torque_limit_nm <= 0.0f || c->joint_torque_limit_nm <= 0.0f) return 0;
    if (c->min_leg_length_m < 0.09f || c->min_leg_length_m >= c->max_leg_length_m) return 0;
    return 1;
}

void Wlc_DefaultConfig(WlcConfig *c)
{
    if (!c) return;
    memset(c, 0, sizeof(*c));
    c->sample_time_s = WLC_GENERATED_SAMPLE_TIME_S;
    c->thigh_length_m = WLC_GENERATED_THIGH_LENGTH_M;
    c->calf_length_m = WLC_GENERATED_CALF_LENGTH_M;
    c->joint_distance_m = WLC_GENERATED_JOINT_DISTANCE_M;
    c->wheel_distance_m = WLC_GENERATED_WHEEL_DISTANCE_M;
    c->body_mass_kg = WLC_GENERATED_BODY_MASS_KG;
    c->max_acceleration_m_s2 = WLC_GENERATED_MAX_ACCELERATION_M_S2;
    c->airborne_force_n = WLC_GENERATED_AIRBORNE_FORCE_N;
    c->wheel_torque_limit_nm = WLC_GENERATED_WHEEL_TORQUE_LIMIT_NM;
    c->joint_torque_limit_nm = WLC_GENERATED_JOINT_TORQUE_LIMIT_NM;
    c->min_leg_length_m = WLC_GENERATED_MIN_LEG_LENGTH_M;
    c->max_leg_length_m = WLC_GENERATED_MAX_LEG_LENGTH_M;
    memcpy(c->lqr_coefficients, WLC_GENERATED_LQR, sizeof(c->lqr_coefficients));
    memcpy(c->mpc_coefficients, WLC_GENERATED_MPC, sizeof(c->mpc_coefficients));
}

int Wlc_Init(WlcContext *ctx, const WlcConfig *config)
{
    if (!ctx || !valid_config(config)) return -1;
    memset(ctx, 0, sizeof(*ctx));
    memcpy(&ctx->config, config, sizeof(*config));
    ctx->burst_start = -10.0f;
    ctx->initialized = 1u;
    return 0;
}

int Wlc_BindPid(WlcContext *ctx,uint32_t index,void *instance,WlcPidCalculateFn calculate,WlcPidResetFn reset)
{
    WlcPidPort *port;
    if(!ctx||!ctx->initialized||index>=WLC_PID_PORT_COUNT||!instance||!calculate)return -1;
    port=&ctx->pid[index];port->instance=instance;port->calculate=calculate;port->reset=reset;
    if(reset)reset(instance);
    return 0;
}

float Wlc_CallPid(WlcContext *ctx,uint32_t index,float measure,float reference)
{
    WlcPidPort *port;
    if(!ctx||index>=WLC_PID_PORT_COUNT)return 0.0f;
    port=&ctx->pid[index];
    return port->calculate?port->calculate(port->instance,measure,reference):0.0f;
}

static int pid_is_bound(const WlcContext *ctx,uint32_t index)
{
    return index<WLC_PID_PORT_COUNT&&ctx->pid[index].calculate!=NULL;
}

void Wlc_Reset(WlcContext *ctx, const WlcInput *initial)
{
    WlcConfig config;
    WlcPidPort ports[WLC_PID_PORT_COUNT];uint32_t index;
    if (!ctx || !ctx->initialized) return;
    memcpy(&config, &ctx->config, sizeof(config));
    memcpy(ports,ctx->pid,sizeof(ports));
    memset(ctx, 0, sizeof(*ctx));
    memcpy(&ctx->config, &config, sizeof(config));
    memcpy(ctx->pid,ports,sizeof(ports));
    for(index=0;index<WLC_PID_PORT_COUNT;++index)if(ctx->pid[index].reset)ctx->pid[index].reset(ctx->pid[index].instance);
    if (initial && isfinite(initial->distance_m) && isfinite(initial->yaw_rad)) {
        ctx->target_distance = initial->distance_m;
        ctx->target_yaw = initial->yaw_rad;
    }
    ctx->burst_start = -10.0f;
    ctx->initialized = 1u;
}

static int link_to_leg(float *p, const WlcConfig *c, float pitch, float pitch_rate)
{
    float x_d, y_d, x_b, y_b, bd, a0, b0, root, x_c, y_c;
    float x_rel, sin32, sin12, sin34, sin03, cos03, sin02, cos02;
    float inv_sin32, inv_length;
    x_d = c->joint_distance_m + c->thigh_length_m * cosf(p[PHI4]);
    y_d = c->thigh_length_m * sinf(p[PHI4]);
    x_b = c->thigh_length_m * cosf(p[PHI1]);
    y_b = c->thigh_length_m * sinf(p[PHI1]);
    bd = (x_d-x_b)*(x_d-x_b) + (y_d-y_b)*(y_d-y_b);
    a0 = 2.0f*c->calf_length_m*(x_d-x_b);
    b0 = 2.0f*c->calf_length_m*(y_d-y_b);
    root = a0*a0 + b0*b0 - bd*bd;
    if (root <= 1.0e-12f) return -1;
    p[PHI2] = 2.0f*atan2f(b0+sqrtf(root), a0+bd);
    x_c = x_b + c->calf_length_m*cosf(p[PHI2]);
    y_c = y_b + c->calf_length_m*sinf(p[PHI2]);
    p[PHI3] = atan2f(y_c-y_d, x_c-x_d);
    x_rel = x_c-c->joint_distance_m*0.5f;
    p[PHI0] = atan2f(y_c,x_rel);
    p[LEG_LENGTH] = sqrtf(x_rel*x_rel+y_c*y_c);
    if (p[LEG_LENGTH] < 0.09f) return -1;
    p[THETA] = p[PHI0]-0.5f*WLC_PI-pitch;
    sin32=sinf(p[PHI3]-p[PHI2]);sin12=sinf(p[PHI1]-p[PHI2]);sin34=sinf(p[PHI3]-p[PHI4]);
    sin03=sinf(p[PHI0]-p[PHI3]);cos03=cosf(p[PHI0]-p[PHI3]);
    sin02=sinf(p[PHI0]-p[PHI2]);cos02=cosf(p[PHI0]-p[PHI2]);
    if (fabsf(sin32)<1.0e-6f) return -1;
    inv_sin32=1.0f/sin32;inv_length=1.0f/p[LEG_LENGTH];
    p[J11]=c->thigh_length_m*sin03*sin12*inv_sin32;
    p[J12]=c->thigh_length_m*sin02*sin34*inv_sin32;
    p[J21]=c->thigh_length_m*cos03*sin12*inv_sin32*inv_length;
    p[J22]=c->thigh_length_m*cos02*sin34*inv_sin32*inv_length;
    p[LEG_RATE]=p[J11]*p[PHI1_RATE]+p[J12]*p[PHI4_RATE];
    p[LEG_ACCEL]=0.05f*((p[LEG_RATE]-p[LAST_LEG_RATE])/c->sample_time_s)+0.95f*p[LEG_ACCEL];
    p[LAST_LEG_RATE]=p[LEG_RATE];
    p[PHI0_RATE]=p[J21]*p[PHI1_RATE]+p[J22]*p[PHI4_RATE];
    p[THETA_RATE]=p[PHI0_RATE]-pitch_rate;
    p[THETA_ACCEL]=0.2f*((p[THETA_RATE]-p[LAST_THETA_RATE])/c->sample_time_s)+0.8f*p[THETA_ACCEL];
    p[LAST_THETA_RATE]=p[THETA_RATE];
    return 0;
}

static float scheduled_gain(const float coefficients[4], float length)
{
    return ((coefficients[0]*length+coefficients[1])*length+coefficients[2])*length+coefficients[3];
}

static void state_feedback(float *p, const WlcConfig *c, float target_distance,
                           float target_velocity, float distance, float velocity,
                           float pitch, float pitch_rate)
{
    float error[6], lqr[2]={0.0f,0.0f}, mpc[2]={0.0f,0.0f};
    uint32_t output, state;
    error[0]=-p[THETA];error[1]=-p[THETA_RATE];error[2]=target_distance-distance;
    error[3]=target_velocity-velocity;error[4]=-pitch;error[5]=-pitch_rate;
    for (output=0;output<2u;++output) {
        for (state=0;state<6u;++state) {
            uint32_t index=output*6u+state;
            if (p[FLYING] && (output==0u || state>=2u)) continue;
            lqr[output]+=scheduled_gain(c->lqr_coefficients[index],p[LEG_LENGTH])*error[state];
            mpc[output]+=scheduled_gain(c->mpc_coefficients[index],p[LEG_LENGTH])*error[state];
        }
    }
    p[HIP_TORQUE]=0.7f*lqr[1]+0.3f*mpc[1];
    p[SUPPORT_FORCE]=lqr[0]; /* Temporary storage for wheel torque until VMC. */
}

static void project_vmc(const float *p, float *back, float *front)
{
    *back=p[J11]*p[SUPPORT_FORCE]+p[J21]*p[HIP_TORQUE];
    *front=p[J12]*p[SUPPORT_FORCE]+p[J22]*p[HIP_TORQUE];
}

static int valid_input(const WlcInput *input, const WlcCommand *command)
{
    if (!input || !command) return 0;
    if (!finite_array((const float *)input, 8u+4u+4u+2u+1u)) return 0;
    if (!isfinite(command->velocity_m_s) || !isfinite(command->yaw_rate_rad_s) ||
        !isfinite(command->leg_length_m)) return 0;
    return 1;
}

int Wlc_Step(WlcContext *ctx, const WlcInput *in, const WlcCommand *cmd,
             WlcOutput *out, WlcDiagnostics *diag)
{
    uint32_t side, status=0u;
    float steer, anti, feedforward, roll_force,yaw_rate_reference;
    if (!out) return -1;
    memset(out,0,sizeof(*out));
    if (diag) memset(diag,0,sizeof(*diag));
    if (!ctx || !ctx->initialized || !valid_input(in,cmd)) {
        out->status_flags=WLC_STATUS_INPUT_INVALID;return -1;
    }
    ctx->target_velocity+=clipf(cmd->velocity_m_s-ctx->target_velocity,
        -ctx->config.max_acceleration_m_s2*ctx->config.sample_time_s,
         ctx->config.max_acceleration_m_s2*ctx->config.sample_time_s);
    ctx->target_distance+=ctx->target_velocity*ctx->config.sample_time_s;
    ctx->target_distance=clipf(ctx->target_distance,in->distance_m-.30f,in->distance_m+.30f);
    ctx->target_yaw+=cmd->yaw_rate_rad_s*ctx->config.sample_time_s;
    for (side=0;side<2u;++side) {
        float *p=ctx->leg[side];uint32_t joint=side*2u;
        p[PHI1]=in->joint_angle_rad[joint];p[PHI4]=in->joint_angle_rad[joint+1u];
        p[PHI1_RATE]=in->joint_rate_rad_s[joint];p[PHI4_RATE]=in->joint_rate_rad_s[joint+1u];
        p[NORMAL_FORCE]=in->normal_force_n[side];p[FLYING]=p[NORMAL_FORCE]<ctx->config.airborne_force_n?1.0f:0.0f;
        if (p[FLYING]) status|=side==0u?WLC_STATUS_LEFT_AIRBORNE:WLC_STATUS_RIGHT_AIRBORNE;
        if (link_to_leg(p,&ctx->config,in->pitch_rad,in->pitch_rate_rad_s)) {
            out->status_flags=status|WLC_STATUS_KINEMATICS_INVALID;return -2;
        }
        state_feedback(p,&ctx->config,ctx->target_distance,ctx->target_velocity,
                       in->distance_m,in->velocity_m_s,in->pitch_rad,in->pitch_rate_rad_s);
    }
    yaw_rate_reference=clipf(5.0f*atan2f(sinf(ctx->target_yaw-in->yaw_rad),cosf(ctx->target_yaw-in->yaw_rad)),-3.0f,3.0f);
    if(pid_is_bound(ctx,WLC_PID_YAW_ANGLE))yaw_rate_reference=clipf(Wlc_CallPid(ctx,WLC_PID_YAW_ANGLE,in->yaw_rad,ctx->target_yaw),-3.0f,3.0f);
    steer=clipf(3.0f*(yaw_rate_reference-in->yaw_rate_rad_s),-1.5f,1.5f);
    if(pid_is_bound(ctx,WLC_PID_YAW_RATE))steer=clipf(Wlc_CallPid(ctx,WLC_PID_YAW_RATE,in->yaw_rate_rad_s,yaw_rate_reference),-1.5f,1.5f);
    if (!ctx->leg[0][FLYING]) ctx->leg[0][SUPPORT_FORCE]-=steer;
    if (!ctx->leg[1][FLYING]) ctx->leg[1][SUPPORT_FORCE]+=steer;
    anti=clipf(-30.0f*(ctx->leg[0][PHI0]-ctx->leg[1][PHI0])
               -2.0f*(ctx->leg[0][PHI0_RATE]-ctx->leg[1][PHI0_RATE]),-60.0f,60.0f);
    if(pid_is_bound(ctx,WLC_PID_ANTI_SPLIT))anti=clipf(Wlc_CallPid(ctx,WLC_PID_ANTI_SPLIT,ctx->leg[0][PHI0]-ctx->leg[1][PHI0],0.0f),-60.0f,60.0f);
    feedforward=(ctx->leg[0][FLYING]&&ctx->leg[1][FLYING])?0.0f:3.0f*steer;
    ctx->leg[0][HIP_TORQUE]+=anti-feedforward;ctx->leg[1][HIP_TORQUE]-=anti-feedforward;
    roll_force=clipf(-4000.0f*in->roll_rad,-400.0f,400.0f);
    if(pid_is_bound(ctx,WLC_PID_ROLL))roll_force=clipf(Wlc_CallPid(ctx,WLC_PID_ROLL,in->roll_rad,0.0f),-400.0f,400.0f);
    if (!(cmd->mode_flags&WLC_MODE_JUMP)) {ctx->jump_locked=0u;ctx->burst=0u;}
    if ((cmd->mode_flags&WLC_MODE_JUMP)&&!ctx->jump_locked&&!ctx->burst&&
        (fabsf(ctx->leg[0][THETA])>.401426f||fabsf(ctx->leg[1][THETA])>.401426f)) {
        ctx->burst=1u;ctx->burst_start=in->time_s;
    }
    for (side=0;side<2u;++side) {
        float *p=ctx->leg[side],target=clipf(cmd->leg_length_m,ctx->config.min_leg_length_m,ctx->config.max_leg_length_m);
        float velocity_reference,error,back,front,lateral;
        if ((cmd->mode_flags&WLC_MODE_JUMP)&&!ctx->jump_locked) target=ctx->config.max_leg_length_m;
        if (ctx->jump_locked) target=ctx->config.min_leg_length_m;
        uint32_t length_port=side==0u?WLC_PID_LEFT_LENGTH:WLC_PID_RIGHT_LENGTH;
        uint32_t speed_port=side==0u?WLC_PID_LEFT_SPEED:WLC_PID_RIGHT_SPEED;
        velocity_reference=clipf(10.0f*(target-p[LEG_LENGTH]),-2.0f,2.0f);
        if(pid_is_bound(ctx,length_port))velocity_reference=clipf(Wlc_CallPid(ctx,length_port,p[LEG_LENGTH],target),-2.0f,2.0f);
        error=velocity_reference-p[LEG_RATE];
        if(!pid_is_bound(ctx,speed_port))ctx->speed_integral[side]=clipf(ctx->speed_integral[side]+50.0f*error*ctx->config.sample_time_s,-50.0f,50.0f);
        /* Replace temporary wheel torque after saving it. */
        out->wheel_torque_nm[side]=clipf(p[SUPPORT_FORCE],-ctx->config.wheel_torque_limit_nm,ctx->config.wheel_torque_limit_nm);
        p[SUPPORT_FORCE]=(pid_is_bound(ctx,speed_port)?clipf(Wlc_CallPid(ctx,speed_port,p[LEG_RATE],velocity_reference),-160.0f,160.0f):
                          clipf(300.0f*error+ctx->speed_integral[side],-160.0f,160.0f))+ctx->config.body_mass_kg*9.81f;
        if (!p[FLYING]) {
            lateral=ctx->config.body_mass_kg*p[LEG_LENGTH]/ctx->config.wheel_distance_m*in->yaw_rate_rad_s*in->velocity_m_s;
            p[SUPPORT_FORCE]+=(side==0u?1.0f:-1.0f)*(roll_force-lateral);
        }
        if (ctx->burst) {
            if (in->time_s-ctx->burst_start<=.2f) {p[SUPPORT_FORCE]=-120.0f;out->wheel_torque_nm[side]=0.0f;}
            else ctx->jump_locked=1u;
        }
        project_vmc(p,&back,&front);
        out->joint_torque_nm[2u*side]=clipf(back,-ctx->config.joint_torque_limit_nm,ctx->config.joint_torque_limit_nm);
        out->joint_torque_nm[2u*side+1u]=clipf(front,-ctx->config.joint_torque_limit_nm,ctx->config.joint_torque_limit_nm);
        if (diag) {
            diag->leg_length_m[side]=p[LEG_LENGTH];diag->virtual_leg_angle_rad[side]=p[THETA];
            diag->leg_rate_m_s[side]=p[LEG_RATE];diag->virtual_leg_rate_rad_s[side]=p[THETA_RATE];
            diag->support_force_n[side]=p[SUPPORT_FORCE];diag->virtual_hip_torque_nm[side]=p[HIP_TORQUE];
        }
    }
    if (ctx->jump_locked) ctx->burst=0u;
    if (ctx->burst) status|=WLC_STATUS_JUMP_BURST;
    if (ctx->jump_locked) status|=WLC_STATUS_JUMP_LOCKED;
    if (cmd->mode_flags&WLC_MODE_ZERO_FORCE) {
        memset(out->wheel_torque_nm,0,sizeof(out->wheel_torque_nm));
        memset(out->joint_torque_nm,0,sizeof(out->joint_torque_nm));
        ctx->target_velocity=0.0f;ctx->target_distance=in->distance_m;ctx->target_yaw=in->yaw_rad;
        ctx->speed_integral[0]=ctx->speed_integral[1]=0.0f;
    }
    if (!finite_array(out->wheel_torque_nm,2u)||!finite_array(out->joint_torque_nm,4u)) {
        memset(out,0,sizeof(*out));out->status_flags=WLC_STATUS_KINEMATICS_INVALID;return -3;
    }
    if (diag) {diag->target_velocity_m_s=ctx->target_velocity;diag->target_distance_m=ctx->target_distance;diag->target_yaw_rad=ctx->target_yaw;}
    out->status_flags=status;
    return 0;
}

uint32_t Wlc_ApiVersion(void)
{
    return WLC_API_VERSION;
}
