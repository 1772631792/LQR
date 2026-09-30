/* Desktop adapter. Original Link2Leg/VMC/fusion kernels are compiled unchanged.
 * See docs/firmware_port.md for hardware substitutions and calibration boundaries. */
#include <math.h>
#include <string.h>
#include "balance.h"
#include "linkNleg.h"
#include "lqr_calc.h"
#define API __declspec(dllexport)
static LinkNPodParam legs[2];
static ChassisParam robot;
static float speed_i[2],target_v,target_dist,target_yaw,delta_t=.001f;
static float burst_start=-10;
static int jump_locked=0,burst=0,leg_topology=0;
static float geom_thigh=THIGH_LEN,geom_calf=CALF_LEN,geom_joint=JOINT_DISTANCE;
static float wheel_limit=8.0f,joint_limit=35.0f;
static float custom_gain[12];static int custom=0;
static float clamp(float x,float lo,float hi){return fminf(hi,fmaxf(lo,x));}

API void FW_Reset(double dt){memset(legs,0,sizeof(legs));memset(&robot,0,sizeof(robot));memset(speed_i,0,sizeof(speed_i));target_v=target_dist=target_yaw=0;delta_t=(float)dt;burst_start=-10;burst=jump_locked=0;custom=0;}
API void FW_SetLegTopology(int topology){leg_topology=topology?1:0;}
API void FW_SetLegGeometry(double thigh,double calf,double joint){geom_thigh=(float)thigh;geom_calf=(float)calf;geom_joint=(float)joint;}
API void FW_SetTorqueLimits(double wheel,double joint){wheel_limit=(float)wheel;joint_limit=(float)joint;}
API void FW_SetGain(const double *k){int i;for(i=0;i<12;++i){custom_gain[i]=(float)k[i];}custom=1;}
API void FW_SetCoefficients(const double *lqr,const double *mpc){int i,j;for(i=0;i<12;++i)for(j=0;j<4;++j){k_lqr[i][j]=(float)lqr[4*i+j];k_mpc[i][j]=(float)mpc[4*i+j];}custom=0;}
API void FW_Gains(double length,double *lqr,double *mpc){int i;for(i=0;i<12;++i){lqr[i]=((k_lqr[i][0]*length+k_lqr[i][1])*length+k_lqr[i][2])*length+k_lqr[i][3];mpc[i]=((k_mpc[i][0]*length+k_mpc[i][1])*length+k_mpc[i][2])*length+k_mpc[i][3];}}
API void FW_Coefficients(double *lqr,double *mpc){int i,j;for(i=0;i<12;++i)for(j=0;j<4;++j){lqr[4*i+j]=k_lqr[i][j];mpc[4*i+j]=k_mpc[i][j];}}
static int Serial2Leg(LinkNPodParam *p,ChassisParam *chassis,float dt)
{
    float q1=p->phi1,q2=p->phi4,q12=q1+q2;
    float x=geom_thigh*sinf(q1)+geom_calf*sinf(q12);
    float y=geom_thigh*cosf(q1)+geom_calf*cosf(q12);
    float dx1=y,dy1=-x,dx2=geom_calf*cosf(q12),dy2=-geom_calf*sinf(q12);
    float l=sqrtf(x*x+y*y),inv_l,inv_l2;
    if(l<.09f)return -1;
    inv_l=1.0f/l;inv_l2=inv_l*inv_l;
    p->leg_len=l;p->phi0=atan2f(y,x);p->theta=p->phi0-.5f*PI-chassis->pitch;
    p->j_11=(x*dx1+y*dy1)*inv_l;p->j_12=(x*dx2+y*dy2)*inv_l;
    p->j_21=(x*dy1-y*dx1)*inv_l2;p->j_22=(x*dy2-y*dx2)*inv_l2;
    p->legd=p->j_11*p->phi1_w+p->j_12*p->phi4_w;
    p->legd_dot=.05f*((p->legd-p->last_legd)/dt)+.95f*p->legd_dot;p->last_legd=p->legd;
    p->phi0_w=p->j_21*p->phi1_w+p->j_22*p->phi4_w;
    p->theta_w=p->phi0_w-chassis->pitch_w;
    p->theta_w_dot=.2f*((p->theta_w-p->last_theta_w)/dt)+.8f*p->theta_w_dot;p->last_theta_w=p->theta_w;
    return 0;
}
static int FiveBar2Leg(LinkNPodParam *p,ChassisParam *chassis,float dt)
{
    float xd=geom_joint+geom_thigh*cosf(p->phi4),yd=geom_thigh*sinf(p->phi4);
    float xb=geom_thigh*cosf(p->phi1),yb=geom_thigh*sinf(p->phi1);
    float bd=(xd-xb)*(xd-xb)+(yd-yb)*(yd-yb),a0=2*geom_calf*(xd-xb),b0=2*geom_calf*(yd-yb);
    float root=a0*a0+b0*b0-bd*bd,xc,yc,xrel,s32,s12,s34,s03,c03,s02,c02;
    if(root<=1e-12f)return -1;
    p->phi2=2*atan2f(b0+sqrtf(root),a0+bd);xc=xb+geom_calf*cosf(p->phi2);yc=yb+geom_calf*sinf(p->phi2);
    p->phi3=atan2f(yc-yd,xc-xd);xrel=xc-.5f*geom_joint;p->phi0=atan2f(yc,xrel);p->leg_len=sqrtf(xrel*xrel+yc*yc);
    if(p->leg_len<.09f)return -1;
    p->theta=p->phi0-.5f*PI-chassis->pitch;
    s32=sinf(p->phi3-p->phi2);s12=sinf(p->phi1-p->phi2);s34=sinf(p->phi3-p->phi4);
    s03=sinf(p->phi0-p->phi3);c03=cosf(p->phi0-p->phi3);s02=sinf(p->phi0-p->phi2);c02=cosf(p->phi0-p->phi2);
    if(fabsf(s32)<1e-6f)return -1;
    p->j_11=geom_thigh*s03*s12/s32;p->j_12=geom_thigh*s02*s34/s32;
    p->j_21=geom_thigh*c03*s12/(s32*p->leg_len);p->j_22=geom_thigh*c02*s34/(s32*p->leg_len);
    p->legd=p->j_11*p->phi1_w+p->j_12*p->phi4_w;p->legd_dot=.05f*((p->legd-p->last_legd)/dt)+.95f*p->legd_dot;p->last_legd=p->legd;
    p->phi0_w=p->j_21*p->phi1_w+p->j_22*p->phi4_w;p->theta_w=p->phi0_w-chassis->pitch_w;
    p->theta_w_dot=.2f*((p->theta_w-p->last_theta_w)/dt)+.8f*p->theta_w_dot;p->last_theta_w=p->theta_w;return 0;
}
/* input: pitch,pitchrate,roll,rollrate,yaw,yawrate,body distance,body velocity,
 *        L(phi1,phi4,w1,w4,normalN), R(same).
 * command: speed m/s,yaw-rate rad/s,height,jump,zero-force,time.
 * output: wheelL,wheelR,backL,frontL,backR,frontR.
 * telemetry per leg: length,theta,legd,theta_w,F,T_hip,normal,fly, then targets v/dist/yaw/jump. */
API int FW_Update(const double *s,const double *cmd,double *out,double *log)
{
    int i,j;float roll_force,steer,anti,ff;
    robot.pitch=(float)s[0];robot.pitch_w=(float)s[1];robot.roll=(float)s[2];robot.roll_w=(float)s[3];robot.yaw=(float)s[4];robot.wz=(float)s[5];robot.dist=(float)s[6];robot.vel=(float)s[7];
    target_v+=clamp((float)cmd[0]-target_v,-MAX_ACC_REF*delta_t,MAX_ACC_REF*delta_t);
    target_dist+=target_v*delta_t;target_dist=clamp(target_dist,robot.dist-.30f,robot.dist+.30f);
    target_yaw+=(float)cmd[1]*delta_t;
    robot.target_v=target_v;robot.target_dist=target_dist;robot.target_yaw=target_yaw;
    for(i=0;i<2;++i){int off=8+5*i;LinkNPodParam *p=&legs[i];p->phi1=(float)s[off];p->phi4=(float)s[off+1];p->phi1_w=(float)s[off+2];p->phi4_w=(float)s[off+3];p->normal_force=(float)s[off+4];p->fly_flag=p->normal_force<20;
        if(leg_topology){if(Serial2Leg(p,&robot,delta_t))return -1;}else if(FiveBar2Leg(p,&robot,delta_t))return -1;
        if(!isfinite(p->leg_len)||p->leg_len<.09f){return -1;}
        CalcLQR_MPC_Fusion(p,&robot);
        if(custom){float err[6]={-p->theta,-p->theta_w,target_dist-robot.dist,target_v-robot.vel,-robot.pitch,-robot.pitch_w};p->T_wheel=p->T_hip=0;for(j=0;j<6;++j){if(!p->fly_flag)p->T_wheel+=custom_gain[j]*err[j];if(!p->fly_flag||j<2)p->T_hip+=custom_gain[j+6]*err[j];}}
    }
    steer=clamp(3*(clamp(5*atan2f(sinf(target_yaw-robot.yaw),cosf(target_yaw-robot.yaw)),-3,3)-robot.wz),-1.5f,1.5f);
    if(!legs[0].fly_flag){legs[0].T_wheel-=steer;}
    if(!legs[1].fly_flag){legs[1].T_wheel+=steer;}
    anti=clamp(-30*(legs[0].phi0-legs[1].phi0)-2*(legs[0].phi0_w-legs[1].phi0_w),-60,60);
    ff=(legs[0].fly_flag&&legs[1].fly_flag)?0:3*steer;
    legs[0].T_hip+=anti-ff;legs[1].T_hip-=anti-ff;
    roll_force=clamp(-4000*robot.roll,-400,400);
    if(!cmd[3]){jump_locked=burst=0;}
    if(cmd[3]&&!jump_locked&&!burst&&(fabsf(legs[0].theta)>.401426f||fabsf(legs[1].theta)>.401426f)){burst=1;burst_start=(float)cmd[5];}
    for(i=0;i<2;++i){LinkNPodParam *p=&legs[i];float target=(float)cmd[2],vref,error;
        if(cmd[3]&&!jump_locked)target=.33f;
        if(jump_locked)target=.15f;
        vref=clamp(10*(target-p->leg_len),-2,2);error=vref-p->legd;
        speed_i[i]=clamp(speed_i[i]+50*error*delta_t,-50,50);
        p->F_leg=clamp(300*error+speed_i[i],-160,160)+BODY_MASS*9.81f;
        if(!p->fly_flag){float lateral=BODY_MASS*p->leg_len/WHEEL_DISTANCE*robot.wz*robot.vel;p->F_leg+=(i==0?1:-1)*(roll_force-lateral);}
        if(burst){if(cmd[5]-burst_start<=.2){p->F_leg=-120;p->T_wheel=0;}else{jump_locked=1;}}
        VMCProject(p);out[i]=clamp(p->T_wheel,-wheel_limit,wheel_limit);out[2+2*i]=clamp(p->T_back,-joint_limit,joint_limit);out[3+2*i]=clamp(p->T_front,-joint_limit,joint_limit);
        log[8*i]=p->leg_len;log[8*i+1]=p->theta;log[8*i+2]=p->legd;log[8*i+3]=p->theta_w;log[8*i+4]=p->F_leg;log[8*i+5]=p->T_hip;log[8*i+6]=p->normal_force;log[8*i+7]=p->fly_flag;
    }
    if(jump_locked)burst=0;
    if(cmd[4]){for(i=0;i<6;++i)out[i]=0;target_v=0;target_dist=robot.dist;target_yaw=robot.yaw;speed_i[0]=speed_i[1]=0;}
    for(i=0;i<6;++i){if(!isfinite(out[i]))return -2;}
    log[16]=target_v;log[17]=target_dist;log[18]=target_yaw;log[19]=burst?1:(jump_locked?2:0);
    return 0;
}
