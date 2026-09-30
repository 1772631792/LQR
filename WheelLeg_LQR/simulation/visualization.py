"""Plots and side-view playback of saved physics states; no control logic."""
import argparse
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle
from config.robot_params import PARAMS
from simulation.design_data import ROOT

plt.rcParams.update({'axes.spines.top': False, 'axes.spines.right': False,
                     'axes.grid': True, 'grid.alpha': 0.22, 'figure.dpi': 110})


def plot_result(result, output):
    t, x, u = result['time'], result['states'], result['force']
    fig, axes = plt.subplots(5, 1, figsize=(10, 10), sharex=True, layout='constrained')
    values = [x[:, 0], x[:, 1], np.rad2deg(x[:, 2]), np.rad2deg(x[:, 3]), u]
    labels = ['Position [m]', 'Velocity [m/s]', 'Pitch [deg]', 'Pitch rate [deg/s]', 'Drive force [N]']
    for i, (ax, value, label) in enumerate(zip(axes, values, labels)):
        if i == 4:
            ax.step(t[:-1], value, where='post', color='#d56b22', linewidth=1.3)
            ax.axhline(PARAMS.output_limit, color='#ab3846', linestyle=':', label='actuator limits')
            ax.axhline(-PARAMS.output_limit, color='#ab3846', linestyle=':')
            ax.legend(loc='upper right', fontsize=8)
        else:
            ax.plot(t, value, color='#1675a9', linewidth=1.4)
            ref = result['reference'][i]
            ax.axhline(np.rad2deg(ref) if i >= 2 else ref, color='gray', linestyle=':', linewidth=1)
        for tick in np.flatnonzero(result['impulses']):
            ax.axvline(t[tick], color='#ab3846', linestyle='--', linewidth=1)
        ax.set_ylabel(label)
    axes[-1].set_xlabel('Simulation time [s]')
    model = 'nonlinear' if bool(result['nonlinear']) else 'linear'
    events = [(float(t[i]), float(result['impulses'][i])) for i in np.flatnonzero(result['impulses'])]
    subtitle = 'No external impulse' if not events else '; '.join(f'{j:g} N s axle impulse at {time:g} s' for time, j in events)
    fig.suptitle(f'Wheel-leg balance | {model} plant | Ts = 1 ms\n{subtitle}', fontsize=14)
    path = Path(output).with_suffix('.png')
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path)
    return fig


def animate_result(result, gif_path=None, fps=30):
    t, x = result['time'], result['states']
    fig, ax = plt.subplots(figsize=(10, 4.8), layout='constrained')
    r, length = PARAMS.wheel_radius, 2*PARAMS.com_height
    left, right = min(-0.45, x[:, 0].min()-0.4), max(0.55, x[:, 0].max()+0.4)
    ax.set(xlim=(left, right), ylim=(-0.08, 0.78), xlabel='Horizontal position [m]', ylabel='Height [m]')
    ax.set_aspect('equal')
    ax.set_title('Side view: two wheels overlap | fixed effective leg height')
    ax.axhline(0, color='#485464', linewidth=2)
    wheel = Circle((0, r), r, facecolor='#dce5ee', edgecolor='#27394b', linewidth=2)
    ax.add_patch(wheel)
    body, = ax.plot([], [], color='#1675a9', linewidth=9, solid_capstyle='round')
    spoke, = ax.plot([], [], color='#27394b', linewidth=2)
    com, = ax.plot([], [], 'o', color='#d56b22', markersize=7, label='Body COM')
    text = ax.text(0.02, 0.97, '', transform=ax.transAxes, va='top', family='monospace', fontsize=10)
    event_text = ax.text(0.98, 0.97, '', transform=ax.transAxes, va='top', ha='right', color='#ab3846')
    ax.legend(loc='lower right')
    events = np.flatnonzero(result['impulses'])
    frame_times = np.arange(0, t[-1], 1/fps)
    indices = np.rint(frame_times/PARAMS.sample_time).astype(int)
    indices = np.unique(np.append(indices, len(t)-1))

    def update(index):
        pos, _, theta, _ = x[index]
        wheel.center = (pos, r)
        rotation = -pos/r
        spoke.set_data([pos, pos+r*np.cos(rotation)], [r, r+r*np.sin(rotation)])
        body.set_data([pos, pos+length*np.sin(theta)], [r, r+length*np.cos(theta)])
        com.set_data([pos+PARAMS.com_height*np.sin(theta)], [r+PARAMS.com_height*np.cos(theta)])
        force = result['force'][min(index, len(result['force'])-1)]
        text.set_text(f't = {t[index]:5.2f} s\np = {pos:+.3f} m\npitch = {np.rad2deg(theta):+.2f} deg\nu = {force:+.2f} N')
        recent = [i for i in events if 0 <= t[index]-t[i] < 0.45]
        event_text.set_text('Axle impulse: '+f'{result["impulses"][recent[-1]]:+g} N s' if recent else '')
        return body, spoke, com, wheel, text, event_text

    animation = FuncAnimation(fig, update, frames=indices, interval=1000/fps, blit=False, repeat=True)
    if gif_path is not None:
        gif_path = Path(gif_path)
        gif_path.parent.mkdir(parents=True, exist_ok=True)
        animation.save(gif_path, writer=PillowWriter(fps=fps))
    return fig, animation


def show(result, output, plot=True, animate=False):
    if plot:
        plot_result(result, output)
    animation = None
    if animate:
        _, animation = animate_result(result)
    plt.show()  # Keep animation referenced until all windows are closed.
    return animation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', nargs='?', type=Path, default=ROOT/'outputs'/'c_baseline.npz')
    parser.add_argument('--gif', type=Path)
    parser.add_argument('--animate', action='store_true')
    parser.add_argument('--no-show', action='store_true')
    args = parser.parse_args()
    with np.load(args.input) as archive:
        result = dict(archive)
    plot_result(result, args.input)
    animation = None
    if args.animate or args.gif:
        _, animation = animate_result(result, args.gif)
    if args.no_show:
        plt.close('all')
    else:
        plt.show()
    return animation


if __name__ == '__main__':
    main()
