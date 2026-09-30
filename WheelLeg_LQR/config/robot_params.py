"""SI units. Design changes require redesigning gains and rebuilding the DLL."""
from dataclasses import dataclass


@dataclass(frozen=True)
class RobotParams:
    base_mass: float = 1.0       # kg, equivalent translating wheel/base mass
    body_mass: float = 5.0       # kg
    com_height: float = 0.25     # m, axle to body center of mass
    body_inertia: float = 0.10   # kg m^2, about body center of mass
    friction: float = 0.10       # N s/m, base viscous damping
    gravity: float = 9.81
    wheel_radius: float = 0.06  # m, visual only in this force-input model
    sample_time: float = 0.001
    output_limit: float = 40.0  # N, total equivalent horizontal drive force


PARAMS = RobotParams()
Q_DIAG = (20.0, 2.0, 300.0, 5.0)
R_VALUE = 0.1
INITIAL_PITCH_DEG = 10.0
DURATION = 10.0
SENSOR_STD = (0.0, 0.0, 0.0, 0.0)  # m, m/s, rad, rad/s; ideal by default
