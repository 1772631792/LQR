#include "lqr_design.h"
#include <math.h>
#include <string.h>

int LQR_Design(const double *a, const double *b, const double *q,
               double r, double ts, double *ad, double *bd, double *p, double *k)
{
    double z[25]={0}, e[25]={0}, term[25]={0}, work[25], next[16], pa[16], pb[4];
    double norm=0.0, scale=1.0;
    int i,j,l,n,squares=0,iteration;
    if (!a || !b || !q || !ad || !bd || !p || !k || !isfinite(r) || r<=0.0 ||
        !isfinite(ts) || ts<0.0001 || ts>0.01) { return -1; }
    for(i=0;i<4;++i) {
        if(!isfinite(q[i]) || q[i]<=0.0 || !isfinite(b[i])) { return -1; }
        for(j=0;j<4;++j) {
            if(!isfinite(a[i*4+j])) { return -1; }
            z[i*5+j]=a[i*4+j]*ts;
        }
        z[i*5+4]=b[i]*ts;
    }
    for(i=0;i<5;++i) { double row=0; for(j=0;j<5;++j) { row+=fabs(z[i*5+j]); } if(row>norm) { norm=row; } }
    while(norm/scale>0.5) { scale*=2.0; ++squares; if(squares>40) {return -2;} }
    for(i=0;i<25;++i) { z[i]/=scale; }
    for(i=0;i<5;++i) { e[i*5+i]=term[i*5+i]=1.0; }
    /* Scaling/squaring Taylor exponential of [[A,B],[0,0]] Ts. */
    for(n=1;n<=24;++n) {
        for(i=0;i<5;++i) for(j=0;j<5;++j) {
            double v=0; for(l=0;l<5;++l) {v+=term[i*5+l]*z[l*5+j];} work[i*5+j]=v/n;
        }
        for(i=0;i<25;++i) {term[i]=work[i];e[i]+=work[i];}
    }
    while(squares-- >0) {
        for(i=0;i<5;++i) for(j=0;j<5;++j) { double v=0;for(l=0;l<5;++l){v+=e[i*5+l]*e[l*5+j];}work[i*5+j]=v; }
        memcpy(e,work,sizeof(e));
    }
    for(i=0;i<4;++i) {bd[i]=e[i*5+4];for(j=0;j<4;++j){ad[i*4+j]=e[i*5+j];p[i*4+j]=(i==j)?q[i]:0.0;}}
    for(iteration=1;iteration<=300000;++iteration) {
        double denominator=r, error=0, magnitude=0;
        for(i=0;i<4;++i) {
            pb[i]=0;for(l=0;l<4;++l){pb[i]+=p[i*4+l]*bd[l];}
            for(j=0;j<4;++j){double v=0;for(l=0;l<4;++l){v+=p[i*4+l]*ad[l*4+j];}pa[i*4+j]=v;}
        }
        for(i=0;i<4;++i){denominator+=bd[i]*pb[i];}
        if(!isfinite(denominator) || denominator<=0) {return -3;}
        for(j=0;j<4;++j){k[j]=0;for(i=0;i<4;++i){k[j]+=bd[i]*pa[i*4+j];}k[j]/=denominator;}
        for(i=0;i<4;++i) for(j=0;j<4;++j) {
            double v=(i==j)?q[i]:0.0;
            for(l=0;l<4;++l){v+=ad[l*4+i]*pa[l*4+j];}
            next[i*4+j]=v-denominator*k[i]*k[j];
        }
        for(i=0;i<4;++i)for(j=i+1;j<4;++j){double average=(next[i*4+j]+next[j*4+i])/2;next[i*4+j]=next[j*4+i]=average;}
        for(i=0;i<16;++i){double delta=next[i]-p[i];error+=delta*delta;magnitude+=next[i]*next[i];p[i]=next[i];}
        if(!isfinite(magnitude)) {return -3;}
        if(sqrt(error/fmax(magnitude,1.0))<1e-12) {break;}
    }
    if(iteration>300000){return -4;}
    /* Recompute gain from the final P. */
    {
        double denominator=r;
        for(i=0;i<4;++i){pb[i]=0;for(l=0;l<4;++l){pb[i]+=p[i*4+l]*bd[l];}denominator+=bd[i]*pb[i];}
        for(j=0;j<4;++j){k[j]=0;for(i=0;i<4;++i){k[j]+=pb[i]*ad[i*4+j];}k[j]/=denominator;}
    }
    return iteration;
}
