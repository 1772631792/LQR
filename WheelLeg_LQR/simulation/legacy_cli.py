"""Run: python main.py. Build first with Ctrl+Shift+B in VS Code."""
import argparse
from pathlib import Path
import numpy as np
from config.leg_params import LEGS

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description='C LQR + Python physics, with five-bar 2D/3D views')
    parser.add_argument('--initial-pitch', type=float, default=10)
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--impulse', type=float, default=0)
    parser.add_argument('--impulse-time', type=float, default=3)
    parser.add_argument('--nonlinear', action='store_true')
    parser.add_argument('--no-show', action='store_true', help='Save PNG/data and exit without GUI')
    parser.add_argument('--leg-demo', action='store_true', help='Separate leg extension kinematics demo; no balance simulation')
    parser.add_argument('--gif', type=Path, help='Optionally export combined 2D/3D animation (can be slow)')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.no_show:
        import matplotlib
        matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from simulation.robot_view import save_views, animate_robot
    from simulation.visualization import plot_result
    output = args.output or ROOT/'outputs'/('five_bar_demo' if args.leg_demo else 'five_bar_balance')
    output.parent.mkdir(parents=True, exist_ok=True)
    heights = None
    if args.leg_demo:
        if not np.isfinite(args.duration) or args.duration <= 0:
            parser.error('--duration must be finite and positive')
        time = np.linspace(0,args.duration,max(2,int(args.duration*100)+1))
        heights = LEGS.nominal_height+0.035*np.sin(2*np.pi*time/3)
        result = dict(time=time, states=np.zeros((len(time),4)))
        np.savez(output.with_suffix('.npz'), **result, heights=heights)
        print('Five-bar leg extension demo: geometric motion only, no dynamics/controller.')
    else:
        from simulation.c_controller import CController
        from simulation.simulator import run, save_result, metrics
        result = run(CController(),duration=args.duration,initial_pitch_deg=args.initial_pitch,
                     nonlinear=args.nonlinear,impulse_ns=args.impulse,impulse_time=args.impulse_time)
        save_result(result, output)
        plot_result(result, output)
        print('C DLL controller + Python four-state plant; fixed five-bar leg geometry.')
        print(metrics(result))
    save_views(result,output,heights,demo=args.leg_demo)
    animation = None
    if not args.no_show or args.gif:
        _, animation = animate_robot(result,heights,demo=args.leg_demo,gif_path=args.gif)
    print(f'2D image: {output.parent / (output.name+"_2d.png")}')
    print(f'3D image: {output.parent / (output.name+"_3d.png")}')
    if args.no_show:
        plt.close('all')
    else:
        plt.show()
    return animation


if __name__ == '__main__':
    main()
