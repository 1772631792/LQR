"""Fixed-step plant/sensor runner. C is the default runtime controller."""
import argparse
from pathlib import Path
import numpy as np
from config.robot_params import PARAMS, INITIAL_PITCH_DEG, DURATION, SENSOR_STD
from simulation.model import PhysicsModel
from simulation.design_data import ROOT


def run(controller, duration=DURATION, initial_pitch_deg=INITIAL_PITCH_DEG,
        nonlinear=False, impulse_ns=0.0, impulse_time=3.0, sensor_std=SENSOR_STD,
        seed=42, reference=None):
    dt = PARAMS.sample_time
    if not np.isfinite(duration) or duration <= 0:
        raise ValueError('duration must be finite and positive')
    steps = round(duration/dt)
    if steps < 1 or not np.isclose(steps*dt, duration, atol=1e-12, rtol=0):
        raise ValueError('duration must be an integer multiple of Ts')
    if not np.isfinite(initial_pitch_deg) or abs(initial_pitch_deg) >= 45:
        raise ValueError('Initial pitch must be finite and within +/-45 degrees.')
    if not np.isfinite(impulse_ns) or not np.isfinite(impulse_time):
        raise ValueError('Impulse parameters must be finite.')
    impulse_step = round(impulse_time/dt)
    if impulse_ns and (not 0 <= impulse_step < steps or not np.isclose(impulse_step*dt, impulse_time, atol=1e-12, rtol=0)):
        raise ValueError('Impulse time must be on a control tick within the simulation.')
    std = np.asarray(sensor_std, dtype=float)
    if std.shape != (4,) or not np.all(np.isfinite(std)) or np.any(std < 0):
        raise ValueError('Sensor standard deviations must be four finite nonnegative values.')
    target = np.zeros(4) if reference is None else np.asarray(reference, dtype=float)
    if target.shape != (4,) or not np.all(np.isfinite(target)):
        raise ValueError('Reference must contain four finite states.')
    rng = np.random.default_rng(seed)
    model = PhysicsModel(nonlinear=nonlinear)
    state = np.array([0.0, 0.0, np.deg2rad(initial_pitch_deg), 0.0])
    states = np.zeros((steps+1, 4))
    measurements = np.zeros((steps, 4))
    forces = np.zeros(steps)
    impulses = np.zeros(steps+1)
    for tick in range(steps):
        if impulse_ns and tick == impulse_step:
            state = model.impulse(state, impulse_ns)
            impulses[tick] = impulse_ns
        states[tick] = state
        measurements[tick] = state + rng.normal(0.0, std)
        # The controller object owns the entire control decision, including saturation.
        forces[tick] = controller.update(measurements[tick], target)
        if not np.isfinite(forces[tick]) or abs(forces[tick]) > PARAMS.output_limit + 1e-5:
            raise RuntimeError('Controller returned an invalid or out-of-range force.')
        state = model.step(state, forces[tick])
        if not np.all(np.isfinite(state)) or abs(state[2]) > np.pi/4:
            raise RuntimeError(f'Model left the configured balance domain at t={(tick+1)*dt:.3f}s.')
    states[-1] = state
    return dict(time=np.arange(steps+1)*dt, states=states, measurements=measurements,
                force=forces, impulses=impulses, reference=target,
                nonlinear=np.array(nonlinear), sensor_std=std)


def metrics(result):
    x, u = result['states'], result['force']
    return {'final_state': x[-1].tolist(), 'peak_pitch_deg': float(np.rad2deg(abs(x[:, 2])).max()),
            'peak_position_m': float(abs(x[:, 0]).max()), 'peak_force_N': float(abs(u).max()),
            'saturation_fraction': float(np.mean(abs(u) >= PARAMS.output_limit-1e-5))}


def save_result(result, output):
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(path.with_suffix('.npz'), **result)
    force = np.append(result['force'], np.nan)  # no command at terminal sample
    measured = np.vstack([result['measurements'], np.full(4, np.nan)])
    np.savetxt(path.with_suffix('.csv'), np.column_stack([result['time'], result['states'], force, result['impulses'], measured]),
               delimiter=',', header='time_s,pos_m,vel_m_s,pitch_rad,pitch_rate_rad_s,force_N,impulse_Ns,measured_pos,measured_vel,measured_pitch,measured_pitch_rate', comments='')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=['c', 'python'], default='c')
    parser.add_argument('--initial-pitch', type=float, default=INITIAL_PITCH_DEG)
    parser.add_argument('--duration', type=float, default=DURATION)
    parser.add_argument('--nonlinear', action='store_true')
    parser.add_argument('--impulse', type=float, default=0.0, help='Horizontal axle impulse in N*s')
    parser.add_argument('--impulse-time', type=float, default=3.0)
    parser.add_argument('--sensor-std', nargs=4, type=float, default=SENSOR_STD)
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--reference-pos', type=float, default=0.0)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs'/'run')
    parser.add_argument('--plot', action='store_true')
    parser.add_argument('--animate', action='store_true')
    args = parser.parse_args()
    if args.backend == 'c':
        from simulation.c_controller import CController
        controller = CController()
    else:
        from simulation.python_reference import PythonReferenceController
        controller = PythonReferenceController()
    result = run(controller, args.duration, args.initial_pitch, args.nonlinear,
                 args.impulse, args.impulse_time, args.sensor_std, args.seed,
                 reference=[args.reference_pos, 0, 0, 0])
    save_result(result, args.output)
    print(f'Backend={args.backend}; Ts={PARAMS.sample_time}s; model={"nonlinear" if args.nonlinear else "linear"}')
    print(metrics(result))
    if args.plot or args.animate:
        from simulation.visualization import show
        show(result, args.output, plot=args.plot, animate=args.animate)


if __name__ == '__main__':
    main()
