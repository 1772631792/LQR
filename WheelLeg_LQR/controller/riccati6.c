/* Six-state / two-input DARE, including the cross cost produced by lqrd.
 * Offline/startup computation, fixed memory. Returns iterations or negative error. */
#include <math.h>
#include <string.h>
__declspec(dllexport) int DARE6(const double *a,const double *b,const double *q,const double *r,const double *cross,double *p,double *k)
{
    double pa[36],pb[12],next[36],g[12],s[4],inv[4];int i,j,l,it;
    memcpy(p,q,36*sizeof(double));
    for(it=1;it<=300000;++it){double change=0,size=0,det;
        for(i=0;i<6;++i){for(j=0;j<6;++j){pa[6*i+j]=0;for(l=0;l<6;++l)pa[6*i+j]+=p[6*i+l]*a[6*l+j];}
          for(j=0;j<2;++j){pb[2*i+j]=0;for(l=0;l<6;++l)pb[2*i+j]+=p[6*i+l]*b[2*l+j];}}
        for(i=0;i<2;++i)for(j=0;j<2;++j){s[2*i+j]=r[2*i+j];for(l=0;l<6;++l)s[2*i+j]+=b[2*l+i]*pb[2*l+j];}
        det=s[0]*s[3]-s[1]*s[2];if(!isfinite(det)||det<=0)return -1;
        inv[0]=s[3]/det;inv[1]=-s[1]/det;inv[2]=-s[2]/det;inv[3]=s[0]/det;
        for(i=0;i<2;++i)for(j=0;j<6;++j){g[6*i+j]=cross[2*j+i];for(l=0;l<6;++l)g[6*i+j]+=b[2*l+i]*pa[6*l+j];}
        for(i=0;i<2;++i)for(j=0;j<6;++j)k[6*i+j]=inv[2*i]*g[j]+inv[2*i+1]*g[6+j];
        for(i=0;i<6;++i)for(j=0;j<6;++j){next[6*i+j]=q[6*i+j]-g[i]*k[j]-g[6+i]*k[6+j];for(l=0;l<6;++l)next[6*i+j]+=a[6*l+i]*pa[6*l+j];}
        for(i=0;i<6;++i)for(j=i+1;j<6;++j){double avg=(next[6*i+j]+next[6*j+i])/2;next[6*i+j]=next[6*j+i]=avg;}
        for(i=0;i<36;++i){double d=next[i]-p[i];change+=d*d;size+=next[i]*next[i];p[i]=next[i];}
        if(!isfinite(size))return -2;
        if(sqrt(change/fmax(size,1.0))<1e-12)return it;
    }return -3;
}
