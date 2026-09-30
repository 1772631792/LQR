"""2D/3D views of two five-bar legs. Geometry does not add physics DOFs."""
from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.patches import Circle, Polygon
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from config.robot_params import PARAMS
from config.leg_params import LEGS
from simulation.five_bar import FiveBarLeg

REAR, FRONT, HIP = '#168aad', '#f08c38', '#526477'


def geometry(position, pitch, height=LEGS.nominal_height):
    leg = FiveBarLeg()
    angles = leg.inverse([0, -height])
    local = leg.forward(angles)
    # Clockwise pitch: positive forward tilt, wheel centre fixed to the ground.
    rot = np.array([[np.cos(pitch), np.sin(pitch)], [-np.sin(pitch), np.cos(pitch)]])
    axle = np.array([position, PARAMS.wheel_radius])
    def world(points):
        return (np.asarray(points)-local[2])@rot.T+axle
    half = LEGS.body_length/2
    body = world([[-half, 0], [half, 0], [half, LEGS.body_height], [-half, LEGS.body_height]])
    # Nominal COM agrees with the existing fixed-height plant.
    com = world([0, PARAMS.com_height-LEGS.nominal_height])
    return world(local), body, com, angles


def draw_2d(ax, state, height, xlimits):
    ax.clear()
    p, _, pitch, _ = state
    points, body, com, angles = geometry(p, pitch, height)
    ax.set(xlim=xlimits, ylim=(-0.025, 0.47), xlabel='Forward x [m]', ylabel='Height z [m]')
    ax.set_aspect('equal')
    ax.set_title('2D side view | left/right legs overlap', fontsize=11)
    ax.axhline(0, color=HIP, lw=2)
    ax.add_patch(Polygon(body, facecolor='#dbe7ee', edgecolor=HIP, lw=2, zorder=2))
    for indices, color in [([0, 1, 2], REAR), ([4, 3, 2], FRONT), ([0, 4], HIP)]:
        segment = points[indices]
        ax.plot(segment[:, 0], segment[:, 1], '-o', color=color, lw=4, ms=6, zorder=4)
    axle = points[2]
    ax.add_patch(Circle(axle, PARAMS.wheel_radius, fc='#343e4c', ec='#17212e', lw=2, zorder=5))
    ax.add_patch(Circle(axle, PARAMS.wheel_radius*0.65, fc='#d2dce5', ec='white', lw=1, zorder=6))
    roll = -p/PARAMS.wheel_radius
    ax.plot([axle[0], axle[0]+0.05*np.cos(roll)], [axle[1], axle[1]+0.05*np.sin(roll)], color='white', lw=2, zorder=7)
    for index, label in enumerate('ABCDE'):
        offset = (6, -16) if label == 'C' else (6, 8)
        ax.annotate(label, points[index], xytext=offset, textcoords='offset points', fontsize=10, weight='bold', zorder=8)
    ax.plot(*com, 'x', ms=9, mew=2, color='#a83257', label='Body COM', zorder=8)
    ax.grid(alpha=0.18)
    ax.legend(loc='upper right', fontsize=8)
    ax.text(0.02, 0.97, f'qA={np.rad2deg(angles[0]):.1f} deg\nqE={np.rad2deg(angles[1]):.1f} deg',
            transform=ax.transAxes, va='top', fontsize=9)


def draw_3d(ax, state, height, xlimits):
    elev, azim = ax.elev, ax.azim
    ax.clear()
    ax.view_init(elev=elev, azim=azim)
    p, _, pitch, _ = state
    points, body, com, _ = geometry(p, pitch, height)
    width = LEGS.body_width/2
    vertices = np.array([[x, y, z] for y in (-width, width) for x, z in body])
    faces = [vertices[ids] for ids in ([0,1,2,3], [4,5,6,7], [0,1,5,4], [1,2,6,5], [2,3,7,6], [3,0,4,7])]
    ax.add_collection3d(Poly3DCollection(faces, facecolor='#d1dfe8', edgecolor=HIP, alpha=0.7, linewidth=1))
    for y, side in [(-LEGS.track_width/2, 'R'), (LEGS.track_width/2, 'L')]:
        for hip in points[[0,4]]:
            ax.plot([hip[0]]*2, [np.sign(y)*width, y], [hip[1]]*2, color=HIP, lw=5)
        for ids, color in [([0,1,2], REAR), ([4,3,2], FRONT), ([0,4], HIP)]:
            q = points[ids]
            ax.plot(q[:,0], np.full(len(ids), y), q[:,1], '-o', color=color, lw=4, ms=4)
        # Cylindrical wheel: tread strip + two circular rims in the x-z plane.
        a = np.linspace(0, 2*np.pi, 40)
        xx = p+PARAMS.wheel_radius*np.cos(a)
        zz = PARAMS.wheel_radius+PARAMS.wheel_radius*np.sin(a)
        ax.plot_surface(np.array([xx, xx]), np.array([np.full_like(a,y-0.022),np.full_like(a,y+0.022)]),
                        np.array([zz, zz]), color='#343e4c', shade=True, alpha=1)
        for rim_y in (y-0.022, y+0.022):
            ax.plot(xx, np.full_like(a,rim_y), zz, color='#17212e', lw=1.5)
            phase = -p/PARAMS.wheel_radius
            ax.plot([p-0.05*np.cos(phase), p+0.05*np.cos(phase)], [rim_y]*2,
                    [PARAMS.wheel_radius-0.05*np.sin(phase), PARAMS.wheel_radius+0.05*np.sin(phase)], color='#a8bbcf', lw=2)
        ax.text(points[0,0], y, points[0,1]+0.035, side, fontsize=10, color=HIP)
    ax.scatter([com[0]], [0], [com[1]], color='#a83257', s=35)
    ax.set(xlim=xlimits, ylim=(-0.28,0.28), zlim=(0,0.47), xlabel='Forward x [m]', ylabel='Left y [m]', zlabel='Height z [m]')
    ax.set_box_aspect((xlimits[1]-xlimits[0],0.56,0.47))
    ax.set_title('3D | twin five-bar legs + two wheels', fontsize=11)
    ax.tick_params(labelsize=8)


def limits(result):
    positions = result['states'][:,0]
    return (float(positions.min()-0.33), float(positions.max()+0.33))


def header(result, index, height, demo):
    state, time = result['states'][index], result['time'][index]
    if demo:
        return f'Five-bar kinematics demo | h={height:.3f} m | t={time:.2f} s (no dynamics)'
    return (f'RoboMaster-style balance | C LQR + Python plant | t={time:.2f} s\n'
            f'pitch={np.rad2deg(state[2]):+.2f} deg | fixed five-bar pose, h={height:.3f} m')


def save_views(result, output, heights=None, demo=False):
    base = Path(output)
    base.parent.mkdir(parents=True, exist_ok=True)
    height = LEGS.nominal_height if heights is None else heights[0]
    for kind in ('2d', '3d'):
        fig = plt.figure(figsize=(9,6))
        fig.subplots_adjust(left=0.09, right=0.90, bottom=0.12, top=0.80)
        ax = fig.add_subplot(111, projection='3d' if kind == '3d' else None)
        if kind == '3d':
            ax.view_init(elev=23, azim=-52)
        (draw_3d if kind == '3d' else draw_2d)(ax, result['states'][0], height, limits(result))
        fig.suptitle(header(result, 0, height, demo), fontsize=12, y=0.98)
        fig.savefig(base.parent/(base.name+f'_{kind}.png'), dpi=150)
        plt.close(fig)


def animate_robot(result, heights=None, demo=False, gif_path=None):
    fig = plt.figure(figsize=(14,6), layout='constrained')
    ax2 = fig.add_subplot(121)
    ax3 = fig.add_subplot(122, projection='3d')
    ax3.view_init(elev=23, azim=-52)
    xlimits = limits(result)
    fps = 20
    times = result['time']
    indices = np.unique(np.append(np.searchsorted(times, np.arange(0,times[-1],1/fps)), len(times)-1))
    title = fig.suptitle('', fontsize=13)

    def update(index):
        height = LEGS.nominal_height if heights is None else heights[index]
        draw_2d(ax2, result['states'][index], height, xlimits)
        draw_3d(ax3, result['states'][index], height, xlimits)
        title.set_text(header(result,index,height,demo))
        return (title,)

    update(0)
    animation = FuncAnimation(fig, update, frames=indices, interval=1000/fps, blit=False, repeat=True)
    if gif_path:
        animation.save(gif_path, writer=PillowWriter(fps=fps))
    return fig, animation
