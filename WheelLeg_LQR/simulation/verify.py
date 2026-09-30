"""Integration checks for the actual DLL, physics and offline design."""
import json
import numpy as np
from config.robot_params import PARAMS
from simulation.model import PhysicsModel, continuous_matrices, verify_model
from simulation.design_lqr import discretize, riccati_iteration
from simulation.design_data import ROOT, load_design
from simulation.python_reference import PythonReferenceController
from simulation.c_controller import CController
from simulation.simulator import run, metrics, save_result
from scipy.linalg import solve_discrete_are


def assert_recovered(result):
    # Check entire final second rather than a single zero crossing.
    tail = result['states'][-1001:] - result['reference']
    assert np.max(abs(tail[:, 0])) < 0.005, 'Position did not settle within 5 mm'
    assert np.max(abs(tail[:, 1])) < 0.01, 'Velocity did not settle within 1 cm/s'
    assert np.max(abs(tail[:, 2])) < np.deg2rad(0.1), 'Pitch did not settle within 0.1 deg'
    assert np.max(abs(tail[:, 3])) < np.deg2rad(0.2), 'Pitch rate did not settle'


def main():
    verify_model()
    data = load_design()
    ad, bd, q, r, k = [np.asarray(data[key]) for key in ('Ad','Bd','Q','R','K')]
    p, _ = riccati_iteration(ad, bd, q, r)
    np.testing.assert_allclose(p, solve_discrete_are(ad, bd, q, r), rtol=1e-7, atol=1e-5)
    assert np.max(abs(np.linalg.eigvals(ad-bd@k))) < 1
    a, b = continuous_matrices()
    exact_ad, exact_bd = discretize(a, b, PARAMS.sample_time)
    np.testing.assert_allclose(ad, exact_ad)
    np.testing.assert_allclose(bd, exact_bd)
    x = np.array([0.2, -0.1, 0.15, 0.3])
    np.testing.assert_allclose(PhysicsModel().step(x, 3), ad@x+bd[:, 0]*3, atol=1e-10, rtol=1e-9)
    c, oracle = CController(), PythonReferenceController()
    np.testing.assert_allclose(list(c.ctrl.K), k[0], rtol=1e-7)
    assert abs(c.ctrl.output_limit-PARAMS.output_limit) < 1e-6
    # Single tick equivalence exercises all four states, references and both saturation limits.
    rng = np.random.default_rng(19)
    for _ in range(1000):
        state, reference = rng.normal(0, 0.2, (2, 4))
        np.testing.assert_allclose(c.update(state, reference), oracle.update(state, reference), atol=2e-5, rtol=2e-6)
    assert c.update([0, 0, 100, 0], [0]*4) == PARAMS.output_limit
    assert c.update([0, 0, -100, 0], [0]*4) == -PARAMS.output_limit
    baseline = run(c)
    python_baseline = run(oracle)
    np.testing.assert_allclose(baseline['states'], python_baseline['states'], atol=2e-6, rtol=2e-5)
    assert_recovered(baseline)
    output_dir = ROOT/'outputs'
    save_result(baseline, output_dir/'c_baseline')
    report = {'baseline': metrics(baseline),
              'c_python_max_state_error': float(abs(baseline['states']-python_baseline['states']).max())}
    cases = {
        'nonlinear': dict(nonlinear=True),
        'disturbance': dict(impulse_ns=0.5, impulse_time=3.0),
        'nonlinear_disturbance': dict(nonlinear=True, impulse_ns=0.5, impulse_time=3.0),
        'negative_tilt': dict(initial_pitch_deg=-10),
        'saturation': dict(initial_pitch_deg=15),
        'position_reference': dict(reference=[0.2, 0, 0, 0]),
        'sensor_noise': dict(sensor_std=[1e-4, 1e-3, 1e-4, 1e-3]),
    }
    for name, options in cases.items():
        result = run(c, **options)
        assert_recovered(result)
        if name == 'saturation':
            assert np.any(abs(result['force']) == PARAMS.output_limit)
        if name == 'disturbance':
            impulse_tick = 3000
            no_impulse_state = PhysicsModel().step(result['states'][impulse_tick-1], result['force'][impulse_tick-1])
            expected = PhysicsModel().impulse(no_impulse_state, 0.5)
            np.testing.assert_allclose(result['states'][impulse_tick], expected, atol=1e-12)
        save_result(result, output_dir/name)
        report[name] = metrics(result)
        print(f'PASS {name}: {report[name]}')
    (output_dir/'verification.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    print('PASS: RK4/ZOH, Riccati, ABI, 1000 C/Python comparisons, closed-loop recovery and disturbances.')
    print('Maximum C/Python trajectory error:', report['c_python_max_state_error'])


if __name__ == '__main__':
    main()
