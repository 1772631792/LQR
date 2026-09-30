"""MATLAB GenerateGains.m replacement: numeric physics + C DARE + polynomial export."""
import json
import ctypes as ct
import numpy as np
from scipy.linalg import expm,solve_discrete_are
from simulation.firmware_native import Firmware
from simulation.chassis_sim import ROOT


def matrices(height):
    # Direct linearization of the three eqA/eqB/eqC equations in matlab.zip.
    L=height/2;M=7.645;mp=1.6164;l=-.016;R=.075;c=.002433/R+.865*R
    inertia=np.array([[-c*height-R*mp*L,c+R*(mp+M),-R*M*l],
                      [mp*L*L/3-mp*L*L,(mp+M)*L+M*L,-M*l*height],
                      [0,-M*l,.16+M*l*l]])
    forcing=np.zeros((3,6));forcing[1,0]=9.81*((mp+M)*L+M*L);forcing[2,4]=M*9.81*l
    aa=np.linalg.solve(inertia,forcing);bb=np.linalg.solve(inertia,[[1,0],[-1,1],[0,1]])
    a=np.zeros((6,6));b=np.zeros((6,2));a[0,1]=a[2,3]=a[4,5]=1
    a[[1,3,5]]=aa;b[[1,3,5]]=bb
    return a,b


def discretize_cost(a,b,q,r,dt):
    # Exact continuous cost discretization, including N, matching MATLAB lqrd.
    f=np.zeros((8,8));f[:6,:6]=a;f[:6,6:]=b
    g=np.zeros((8,8));g[:6,:6]=q;g[6:,6:]=r
    block=np.block([[-f.T,g],[np.zeros((8,8)),f]])
    e=expm(block*dt);transition=e[8:,8:];cost=transition.T@e[:8,8:]
    cost=(cost+cost.T)/2
    return transition[:6,:6].copy(),transition[:6,6:].copy(),cost[:6,:6].copy(),cost[6:,6:].copy(),cost[:6,6:].copy()


def finite_mpc(ad,bd,q,r,horizon=5):
    phi=np.vstack([np.linalg.matrix_power(ad,k+1) for k in range(horizon)])
    gamma=np.zeros((6*horizon,2*horizon))
    for i in range(horizon):
        for j in range(i+1):gamma[6*i:6*i+6,2*j:2*j+2]=np.linalg.matrix_power(ad,i-j)@bd
    qb=np.kron(np.eye(horizon),q);rb=np.kron(np.eye(horizon),r)
    return np.linalg.solve(gamma.T@qb@gamma+rb,gamma.T@qb@phi)[:2]


def generate(output=None,q_values=None,r_values=None):
    fw=Firmware();ptr=np.ctypeslib.ndpointer(dtype=np.float64,flags='C_CONTIGUOUS')
    solver=fw.dll.DARE6;solver.argtypes=[ptr]*7;solver.restype=ct.c_int
    heights=np.arange(30)*.01+.10;q=np.diag(q_values if q_values is not None else [480.,240,200,600,2000,50]);r=np.diag(r_values if r_values is not None else [2.5,.25]);rows=[];mpcs=[];records=[]
    for h in heights:
        a,b=matrices(h);ad,bd,qd,rd,nd=discretize_cost(a,b,q,r,.001);p=np.empty((6,6));k=np.empty((2,6))
        iterations=solver(ad,bd,qd,rd,nd,p,k)
        if iterations<0:raise RuntimeError(f'C DARE6 failed at h={h}: {iterations}')
        reference=solve_discrete_are(ad,bd,qd,rd,s=nd)
        np.testing.assert_allclose(p,reference,rtol=2e-7,atol=2e-5)
        eigen=np.linalg.eigvals(ad-bd@k)
        assert np.max(abs(eigen))<1
        rows.append(k.ravel());mpcs.append(finite_mpc(ad,bd,q,r).ravel())
        records.append(dict(height=h,A=a.tolist(),B=b.tolist(),Ad=ad.tolist(),Bd=bd.tolist(),Qd=qd.tolist(),Rd=rd.tolist(),Nd=nd.tolist(),P=p.tolist(),K=k.tolist(),iterations=iterations,eigenvalues=[[float(v.real),float(v.imag)] for v in eigen],spectral_radius=float(max(abs(eigen)))))
    coefficients=np.polyfit(heights,np.array(rows),3).T;mpc_coefficients=np.polyfit(heights,np.array(mpcs),3).T
    fitted_radius=[];fusion_radius=[]
    # Validate the actual float coefficient schedule, not just the ideal node gains.
    for h in np.linspace(.14,.33,100):
        a,b=matrices(h);ad,bd,*_=discretize_cost(a,b,q,r,.001)
        powers=np.array([h**3,h*h,h,1]);gain=(coefficients.astype(np.float32)@powers).reshape(2,6)
        mpc=(mpc_coefficients.astype(np.float32)@powers).reshape(2,6)
        fitted_radius.append(float(max(abs(np.linalg.eigvals(ad-bd@gain)))))
        gain[1]=.7*gain[1]+.3*mpc[1]
        fusion_radius.append(float(max(abs(np.linalg.eigvals(ad-bd@gain)))))
    if max(fitted_radius+fusion_radius)>=1:raise RuntimeError('Fitted/fused gain schedule is unstable in the 0.14–0.33 m operating range; adjust Q/R.')
    original,original_mpc=np.empty((12,4)),np.empty((12,4));fw.dll.FW_Coefficients(original,original_mpc)
    report=dict(source='balance_chassis-main/matlab.zip: GenerateGains.m',Q=q.tolist(),R=r.tolist(),dt=.001,records=records,
                lqr_coefficients=coefficients.tolist(),mpc_coefficients=mpc_coefficients.tolist(),
                fitted_max_spectral_radius=max(fitted_radius),fusion_max_spectral_radius=max(fusion_radius),
                source_lqr_max_coefficient_error=float(abs(original-coefficients).max()),source_mpc_max_coefficient_error=float(abs(original_mpc-mpc_coefficients).max()))
    output=output or ROOT/'outputs'/'source_design';output.mkdir(parents=True,exist_ok=True)
    (output/'design.json').write_text(json.dumps(report,indent=2),encoding='utf-8')
    def c_matrix(name,values):return f'static const float {name}[12][4] = {{\n'+',\n'.join('    {'+', '.join(f'{v:.9e}f' for v in row)+'}' for row in values)+'\n};\n'
    (output/'regenerated_gains.h').write_text(c_matrix('k_lqr',coefficients)+c_matrix('k_mpc',mpc_coefficients),encoding='utf-8')
    print('MATLAB-free generation passed: 30 nodes, C DARE6, exact lqrd cost, 5-step unconstrained MPC, cubic fitting.')
    print('Source LQR coefficient max error:',report['source_lqr_max_coefficient_error'])
    print('Source MPC coefficient max error:',report['source_mpc_max_coefficient_error'])
    fw.close()
    return report


if __name__=='__main__':generate()
