"""Upper-computer entry point. Ctrl+Shift+B builds C; python main.py opens the GUI."""
import argparse
from pathlib import Path
import sys


def main():
    if '--leg-demo' in sys.argv:
        from simulation.legacy_cli import main as legacy_main
        return legacy_main()
    parser=argparse.ArgumentParser(description='C chassis algorithms + Python simulation upper computer')
    parser.add_argument('--headless','--no-show',action='store_true',dest='headless')
    parser.add_argument('--duration',type=float,default=10)
    parser.add_argument('--control-dt',type=float,default=.001)
    parser.add_argument('--physics-dt',type=float,default=.001)
    parser.add_argument('--initial-pitch',type=float,default=10)
    parser.add_argument('--impulse',type=float,default=.5)
    parser.add_argument('--impulse-time',type=float,default=4)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'outputs'/'chassis')
    args=parser.parse_args()
    from simulation.chassis_sim import Settings,simulate,save_run
    settings=Settings(duration=args.duration,control_dt=args.control_dt,physics_dt=args.physics_dt,
                      initial_pitch=args.initial_pitch,impulse=args.impulse,impulse_time=args.impulse_time)
    if not args.headless:
        from simulation.host_app import launch
        launch(settings)
        return
    import matplotlib
    matplotlib.use('Agg')
    from matplotlib.figure import Figure
    from simulation.chassis_view import curves,view2d,view3d
    result=simulate(settings);save_run(result,args.output)
    for legs,name in [(False,'balance.png'),(True,'legs_motors.png')]:
        fig=Figure(figsize=(12,8));curves(fig,result,legs);fig.savefig(args.output/name,dpi=140)
    limits=(float(result['states'][:,0].min()-.35),float(result['states'][:,0].max()+.35))
    fig=Figure(figsize=(12,6));view2d(fig.subplots(1,2),result['states'][0],limits);fig.tight_layout();fig.savefig(args.output/'robot_2d.png',dpi=140)
    fig=Figure(figsize=(10,7));ax=fig.add_subplot(111,projection='3d');ax.view_init(22,-55)
    view3d(ax,result['states'][0],limits);fig.savefig(args.output/'robot_3d.png',dpi=140)
    print(f'C LQR design + chassis control: {settings.duration:g}s simulated in {result["elapsed"]:.3f}s')
    print('Final state:',result['states'][-1]);print('Export:',args.output)


if __name__=='__main__':
    main()
