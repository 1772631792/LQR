"""Scientific/integration verification. SciPy used only here, never by host runtime."""
from dataclasses import replace
import threading
import numpy as np
from scipy.linalg import solve_discrete_are
from simulation.chassis_sim import NativeChassis,Settings,simulate,derivative,ROOT,save_run
from simulation.design_lqr import discretize
from simulation.five_bar import FiveBarLeg


def main():
    native=NativeChassis()
    try:
        for ts in (.0005,.001,.002,.005):
            settings=Settings(control_dt=ts,physics_dt=ts)
            designs=native.initialize(settings)
            for item in designs:
                a,b=np.array(item['A']),np.array(item['B']);ad,bd=discretize(a,b,ts)
                p=solve_discrete_are(ad,bd,np.diag(settings.q),np.array([[settings.r]]))
                k=np.linalg.solve([[settings.r]]+bd.T@p@bd,bd.T@p@ad)
                np.testing.assert_allclose(item['Ad'],ad,atol=1e-12)
                np.testing.assert_allclose(item['Bd'],bd[:,0],atol=1e-12)
                np.testing.assert_allclose(item['P'],p,rtol=2e-7,atol=2e-5)
                np.testing.assert_allclose(item['K'],k[0],rtol=2e-7)
            for h in np.linspace(.16,.24,41):
                item=native.design(h,settings);ad,bd=np.array(item['Ad']),np.array(item['Bd'])
                gains=np.array([row['K'] for row in designs]);k=np.array([np.interp(h,[.16,.20,.24],gains[:,i]) for i in range(4)])
                assert np.max(abs(np.linalg.eigvals(ad-np.outer(bd,k))))<1
        settings=Settings();native.initialize(settings)
        state=np.array([0,0,0,0,.2,0,0,0,0,0.],dtype=np.float64);ref=np.array([0,.2,0,0.]);out=np.empty(10)
        native.update(state,ref,out)
        np.testing.assert_allclose(out[1:3],[24.525]*2,atol=1e-5)
        leg=FiveBarLeg();j=leg.jacobian(leg.inverse([0,-.2]))
        np.testing.assert_allclose(out[6:8],j.T@np.array([0,-out[1]]),atol=1e-5)
        np.testing.assert_allclose(derivative(state,out),0,atol=2e-6)
        native.initialize(replace(settings,joint_limit=1))
        native.update(state,ref,out)
        assert np.max(abs(out[6:]))<=1.000001 and max(out[1:3])<24.525
    finally:native.close()
    baseline=simulate(Settings());s=baseline['states'];out=baseline['outputs'];tail=s[-1000:]
    assert max(abs(tail[:,2]))<np.deg2rad(.1)
    assert max(abs(tail[:,0]))<.005
    assert max(abs(tail[:,4]-.23))<.001
    assert max(abs(tail[:,6]))<np.deg2rad(.1)
    assert max(abs(tail[:,8]-np.deg2rad(15)))<np.deg2rad(.3)
    assert max(abs(out[:,4:6]).flat)<=1.50001 and max(abs(out[:,6:]).flat)<=12.0001
    fine=simulate(replace(Settings(),physics_dt=.0005))
    np.testing.assert_allclose(baseline['states'],fine['states'],atol=2e-6,rtol=3e-5)
    slower=simulate(replace(Settings(),control_dt=.002,physics_dt=.001))
    assert abs(slower['states'][-1,2])<np.deg2rad(.1)
    repeat=simulate(Settings())
    np.testing.assert_array_equal(baseline['states'],repeat['states'])
    cancel=threading.Event();cancel.set()
    try:simulate(Settings(),cancel=cancel)
    except InterruptedError:pass
    else:raise AssertionError('Cancellation ignored')
    save_run(baseline,ROOT/'outputs'/'chassis')
    print('PASS: C ZOH/DARE vs SciPy, 164 interpolated stability points, VMC/force derating,')
    print('      height/roll/yaw/balance/impulse recovery, integration convergence, period change, reset, cancel.')
    print('Default simulation wall time:',baseline['elapsed'])


if __name__=='__main__':main()
