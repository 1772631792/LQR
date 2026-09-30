/* Independent two-link serial-leg kinematics. */
#ifndef WLC_KINEMATICS_H
#define WLC_KINEMATICS_H
static int leg_to_virtual(float *p,const WlcConfig *c,float pitch,float pitch_rate)
{
    float q1=p[PHI1],q2=p[PHI4],q12=q1+q2;
    float x=c->thigh_length_m*sinf(q1)+c->calf_length_m*sinf(q12);
    float y=c->thigh_length_m*cosf(q1)+c->calf_length_m*cosf(q12);
    float dx1=y,dy1=-x,dx2=c->calf_length_m*cosf(q12),dy2=-c->calf_length_m*sinf(q12),invl,invl2;
    p[LEG_LENGTH]=sqrtf(x*x+y*y);
    if(p[LEG_LENGTH]<.09f)return -1;
    invl=1.0f/p[LEG_LENGTH];invl2=invl*invl;
    p[PHI0]=atan2f(y,x);p[THETA]=p[PHI0]-.5f*WLC_PI-pitch;
    p[J11]=(x*dx1+y*dy1)*invl;p[J12]=(x*dx2+y*dy2)*invl;p[J21]=(x*dy1-y*dx1)*invl2;p[J22]=(x*dy2-y*dx2)*invl2;
    p[LEG_RATE]=p[J11]*p[PHI1_RATE]+p[J12]*p[PHI4_RATE];p[LEG_ACCEL]=.05f*((p[LEG_RATE]-p[LAST_LEG_RATE])/c->sample_time_s)+.95f*p[LEG_ACCEL];p[LAST_LEG_RATE]=p[LEG_RATE];
    p[PHI0_RATE]=p[J21]*p[PHI1_RATE]+p[J22]*p[PHI4_RATE];p[THETA_RATE]=p[PHI0_RATE]-pitch_rate;
    p[THETA_ACCEL]=.2f*((p[THETA_RATE]-p[LAST_THETA_RATE])/c->sample_time_s)+.8f*p[THETA_ACCEL];p[LAST_THETA_RATE]=p[THETA_RATE];return 0;
}
#endif
