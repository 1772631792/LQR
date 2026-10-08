"""Simplified RMUC 2026 arena geometry for MuJoCo control training.

The rulebook drawings are construction references, not a public CAD model.
This module therefore keeps the official overall scale and the terrain features
that matter to a mobile chassis, while deliberately simplifying small props.
Coordinates are metres; +x points from the red half toward the blue half.
"""
import math


ARENA_LENGTH = 28.0
ARENA_WIDTH = 15.0
WALL_HEIGHT = 2.4
RULEBOOK_VERSION = "RMUC 2026 V2.2.0 index; terrain drawings checked in V2.1.0"

SPAWNS = {
    # x, y, supporting-surface height, yaw (rad)
    "RM地图": (-8.2, 3.8, .20, 0.0),
    # Compatibility with recordings made before the UI label was renamed.
    "飞坡": (-8.2, 3.8, .20, 0.0),
    "起伏路": (-10.5, -5.2, 0.0, 0.0),
    "中央高地": (-7.2, 0.0, 0.0, 0.0),
    "平地": (-11.5, 6.45, 0.0, 0.0),
}


def spawn_pose(scene):
    """Return a safe training spawn, defaulting to the full RM map."""
    return SPAWNS.get(scene, SPAWNS["RM地图"])


def _quat_y(angle):
    return f'{math.cos(angle/2):.10g} 0 {math.sin(angle/2):.10g} 0'


def _box(name, pos, size, rgba, *, collide=True, quat=None):
    contact = 'contype="1" conaffinity="1"' if collide else 'contype="0" conaffinity="0"'
    rotation = f' quat="{quat}"' if quat else ''
    return (f'<geom name="{name}" type="box" pos="{pos[0]:.6g} {pos[1]:.6g} {pos[2]:.6g}" '
            f'size="{size[0]:.6g} {size[1]:.6g} {size[2]:.6g}"{rotation} '
            f'rgba="{rgba}" {contact}/>')


def _cylinder(name, pos, radius, half_height, rgba, *, collide=True):
    contact = 'contype="1" conaffinity="1"' if collide else 'contype="0" conaffinity="0"'
    return (f'<geom name="{name}" type="cylinder" pos="{pos[0]:.6g} {pos[1]:.6g} {pos[2]:.6g}" '
            f'size="{radius:.6g} {half_height:.6g}" rgba="{rgba}" {contact}/>')


def _ramp(name, center_xy, length, width, low, high, rgba, rising_positive_x=True):
    """A thin tilted collision box whose top follows the requested slope."""
    angle = math.atan2(high-low, length)
    if not rising_positive_x:
        angle = -angle
    # Rotation about +y makes the +x end lower, hence the sign inversion.
    quat = _quat_y(-angle)
    thickness = .055
    z = (low+high)/2-thickness/2
    return _box(name, (center_xy[0], center_xy[1], z), (length/2, width/2, thickness/2),
                rgba, quat=quat)


def arena_xml():
    """Return worldbody geoms for a symmetric, driveable RMUC 2026 arena."""
    dark = '.105 .125 .145 1'
    road = '.22 .245 .255 1'
    highland = '.255 .285 .295 1'
    edge = '.38 .41 .42 1'
    red = '.82 .08 .11 1'
    blue = '.08 .22 .78 1'
    geoms = [
        _box('arena_floor', (0, 0, -.035), (14, 7.5, .035), dark),
        # Official barrier upper edge is 2.4 m above the battlefield floor.
        _box('wall_north', (0, 7.55, WALL_HEIGHT/2), (14.1, .05, WALL_HEIGHT/2), '.035 .04 .045 1'),
        _box('wall_south', (0, -7.55, WALL_HEIGHT/2), (14.1, .05, WALL_HEIGHT/2), '.035 .04 .045 1'),
        _box('wall_red', (-14.05, 0, WALL_HEIGHT/2), (.05, 7.5, WALL_HEIGHT/2), '.035 .04 .045 1'),
        _box('wall_blue', (14.05, 0, WALL_HEIGHT/2), (.05, 7.5, WALL_HEIGHT/2), '.035 .04 .045 1'),
    ]

    # Central highland: 0.40 m deck with the rulebook's 10.5 degree approaches.
    center_height = .40
    center_half_length = 3.0
    center_half_width = 4.15
    center_run = center_height/math.tan(math.radians(10.5))
    geoms += [
        _box('central_highland', (0, 0, center_height/2),
             (center_half_length, center_half_width, center_height/2), highland),
        _ramp('central_ramp_red', (-center_half_length-center_run/2, 0), center_run,
              2*center_half_width, 0, center_height, highland, True),
        _ramp('central_ramp_blue', (center_half_length+center_run/2, 0), center_run,
              2*center_half_width, center_height, 0, highland, True),
    ]

    # Road decks and fly ramps.  The launch face follows the documented 17 degrees,
    # with a 0.20 -> 0.553 m rise and an approximately 0.65 m gap.
    geoms += [
        _box('road_red', (-9.0, 3.8, .10), (3.0, .60, .10), road),
        _box('road_blue', (9.0, -3.8, .10), (3.0, .60, .10), road),
    ]
    fly_low, fly_high = .203, .553
    fly_run = (fly_high-fly_low)/math.tan(math.radians(17))
    geoms += [
        _ramp('launch_ramp', (-5.45-fly_run/2, 3.8), fly_run, 1.145,
              fly_low, fly_high, road, True),
        _box('ramp_platform', (-4.18, 3.8, .10), (.45, .5725, .10), road),
        _ramp('launch_ramp_blue', (5.45+fly_run/2, -3.8), fly_run, 1.145,
              fly_high, fly_low, road, True),
        _box('ramp_platform_blue', (4.18, -3.8, .10), (.45, .5725, .10), road),
    ]

    # 70 mm high, 240 mm pitch undulating road sections.
    for side, x0, y, color in (('red', -10.0, -5.2, red), ('blue', 10.0, 5.2, blue)):
        direction = 1 if side == 'red' else -1
        geoms.append(_box(f'rough_lane_{side}', (x0+direction*1.45, y, .012),
                          (1.65, .72, .012), road))
        for i in range(13):
            x = x0+direction*(.12+i*.24)
            geoms.append(_box(f'rough_{side}_{i:02d}', (x, y, .047),
                              (.042, .70, .035), edge, quat=_quat_y(math.radians(28)*direction)))

    # Trapezoid highlands: reduced polygonal detail, correct 0.20-0.40 m height scale.
    for side, x, y, color in (('red', -8.2, -3.35, red), ('blue', 8.2, 3.35, blue)):
        geoms += [
            _box(f'trapezoid_{side}_low', (x, y, .10), (1.15, 1.45, .10), highland),
            _box(f'trapezoid_{side}_high', (x+(1.15 if side == 'red' else -1.15), y, .20),
                 (.72, 1.45, .20), highland),
        ]

    # Bases, outposts and fortress pads provide useful navigation obstacles.
    for side, sign, color in (('red', -1, red), ('blue', 1, blue)):
        geoms += [
            _cylinder(f'base_pad_{side}', (sign*10.8, 0, .06), 1.18, .06, edge),
            _cylinder(f'base_{side}', (sign*10.8, 0, .48), .46, .42, color),
            _cylinder(f'outpost_{side}', (sign*5.25, -sign*1.75, .43), .38, .43, color),
            _box(f'fortress_{side}', (sign*8.1, -sign*4.8, .12), (1.0, .75, .12), highland),
            _box(f'spawn_mark_{side}', (sign*10.8, 0, .123), (1.45, .025, .003), color, collide=False),
        ]

    # Low coloured centre lines are visual only and do not perturb wheel contacts.
    geoms += [
        _box('red_half_marker', (-7.0, 7.38, .006), (6.8, .035, .006), red, collide=False),
        _box('blue_half_marker', (7.0, -7.38, .006), (6.8, .035, .006), blue, collide=False),
        _box('centre_line', (0, 0, .006), (.025, 7.3, .006), '.75 .75 .70 1', collide=False),
    ]
    return '\n        '.join(geoms)


def collision_geom_names():
    """Stable feature names used by regression tests and contact telemetry."""
    return ('arena_floor', 'launch_ramp', 'ramp_platform', 'central_highland',
            'central_ramp_red', 'central_ramp_blue', 'rough_red_00', 'rough_blue_00')
