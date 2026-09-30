/* Independent parallel five-bar leg kinematics. */
#ifndef WLC_KINEMATICS_H
#define WLC_KINEMATICS_H
static int leg_to_virtual(float *p,const WlcConfig *c,float pitch,float pitch_rate)
{
    float xd,yd,xb,yb,bd,a0,b0,root,xc,yc,xrel;
    float s32,s12,s34,s03,c03,s02,c02,inv32,invl;
    xd=c->joint_distance_m+c->thigh_length_m*cosf(p[PHI4]);yd=c->thigh_length_m*sinf(p[PHI4]);
    xb=c->thigh_length_m*cosf(p[PHI1]);yb=c->thigh_length_m*sinf(p[PHI1]);
    bd=(xd-xb)*(xd-xb)+(yd-yb)*(yd-yb);a0=2.0f*c->calf_length_m*(xd-xb);b0=2.0f*c->calf_length_m*(yd-yb);
    root=a0*a0+b0*b0-bd*bd;if(root<=1.0e-12f)return -1;
    p[PHI2]=2.0f*atan2f(b0+sqrtf(root),a0+bd);xc=xb+c->calf_length_m*cosf(p[PHI2]);yc=yb+c->calf_length_m*sinf(p[PHI2]);
    p[PHI3]=atan2f(yc-yd,xc-xd);xrel=xc-c->joint_distance_m*.5f;p[PHI0]=atan2f(yc,xrel);p[LEG_LENGTH]=sqrtf(xrel*xrel+yc*yc);
    if(p[LEG_LENGTH]<.09f)return -1;
    p[THETA]=p[PHI0]-.5f*WLC_PI-pitch;
    s32=sinf(p[PHI3]-p[PHI2]);s12=sinf(p[PHI1]-p[PHI2]);s34=sinf(p[PHI3]-p[PHI4]);
    s03=sinf(p[PHI0]-p[PHI3]);c03=cosf(p[PHI0]-p[PHI3]);s02=sinf(p[PHI0]-p[PHI2]);c02=cosf(p[PHI0]-p[PHI2]);
    if(fabsf(s32)<1.0e-6f)return -1;
    inv32=1.0f/s32;invl=1.0f/p[LEG_LENGTH];
    p[J11]=c->thigh_length_m*s03*s12*inv32;p[J12]=c->thigh_length_m*s02*s34*inv32;
    p[J21]=c->thigh_length_m*c03*s12*inv32*invl;p[J22]=c->thigh_length_m*c02*s34*inv32*invl;
    p[LEG_RATE]=p[J11]*p[PHI1_RATE]+p[J12]*p[PHI4_RATE];p[LEG_ACCEL]=.05f*((p[LEG_RATE]-p[LAST_LEG_RATE])/c->sample_time_s)+.95f*p[LEG_ACCEL];p[LAST_LEG_RATE]=p[LEG_RATE];
    p[PHI0_RATE]=p[J21]*p[PHI1_RATE]+p[J22]*p[PHI4_RATE];p[THETA_RATE]=p[PHI0_RATE]-pitch_rate;
    p[THETA_ACCEL]=.2f*((p[THETA_RATE]-p[LAST_THETA_RATE])/c->sample_time_s)+.8f*p[THETA_ACCEL];p[LAST_THETA_RATE]=p[THETA_RATE];return 0;
}
#endif
