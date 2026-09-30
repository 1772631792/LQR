"""Planar A-B-C-D-E-A five-bar: A/E driven hips, B/D knees, C wheel axle.

Local coordinates: x forward, z up; hip midpoint is (0, 0).
Joint angles are measured counterclockwise from +x. Lower assembly branch only
for the inverse solver; forward() exposes both closure branches explicitly.
"""
import numpy as np
from config.leg_params import LEGS


def circle_intersections(c0, r0, c1, r1):
    c0, c1 = np.asarray(c0, dtype=float), np.asarray(c1, dtype=float)
    delta = c1-c0
    distance = np.linalg.norm(delta)
    if not np.isfinite(distance) or distance < 1e-10:
        raise ValueError('Coincident circle centres: undefined linkage branch.')
    along = (r0*r0-r1*r1+distance*distance)/(2*distance)
    height2 = r0*r0-along*along
    if height2 <= 1e-12:
        raise ValueError('Unreachable or singular five-bar configuration.')
    direction = delta/distance
    midpoint = c0+along*direction
    offset = np.sqrt(height2)*np.array([-direction[1], direction[0]])
    return midpoint+offset, midpoint-offset


class FiveBarLeg:
    def __init__(self, params=LEGS):
        self.params = params
        if min(params.hip_spacing, params.upper_length, params.lower_length) <= 0:
            raise ValueError('Link lengths must be positive.')
        self.a = np.array([-params.hip_spacing/2, 0.0])
        self.e = np.array([params.hip_spacing/2, 0.0])

    def forward(self, angles, branch=-1):
        angles = np.asarray(angles, dtype=float)
        if angles.shape != (2,) or not np.all(np.isfinite(angles)) or branch not in (-1, 1):
            raise ValueError('Expected two finite joint angles and branch +/-1.')
        u, v = angles
        b = self.a+self.params.upper_length*np.array([np.cos(u), np.sin(u)])
        d = self.e+self.params.upper_length*np.array([np.cos(v), np.sin(v)])
        plus, minus = circle_intersections(b, self.params.lower_length, d, self.params.lower_length)
        c = plus if branch == 1 else minus
        return np.array([self.a, b, c, d, self.e])

    def inverse(self, foot):
        foot = np.asarray(foot, dtype=float)
        if foot.shape != (2,) or not np.all(np.isfinite(foot)) or foot[1] >= 0:
            raise ValueError('Wheel target must be a finite (x,z) below the hip.')
        p = self.params
        # Outward knees: rear knee to the rear, front knee to the front.
        b = min(circle_intersections(self.a, p.upper_length, foot, p.lower_length), key=lambda v: v[0])
        d = max(circle_intersections(self.e, p.upper_length, foot, p.lower_length), key=lambda v: v[0])
        angles = np.array([np.arctan2(*(b-self.a)[::-1]), np.arctan2(*(d-self.e)[::-1])])
        if not np.allclose(self.forward(angles)[2], foot, atol=1e-9, rtol=0):
            raise ValueError('Target belongs to another assembly branch.')
        return angles

    def jacobian(self, angles):
        """Differentiated rod constraints: foot_velocity = J @ joint_velocity."""
        a, b, c, d, e = self.forward(angles)
        cb, cd = c-b, c-d
        tangent_b = np.array([-(b-a)[1], (b-a)[0]])
        tangent_d = np.array([-(d-e)[1], (d-e)[0]])
        constraint = np.vstack([cb, cd])
        if np.linalg.cond(constraint) > 1e8:
            raise ValueError('Five-bar Jacobian is singular.')
        return np.linalg.solve(constraint, np.diag([cb@tangent_b, cd@tangent_d]))


def verify():
    leg = FiveBarLeg()
    for x in np.linspace(-0.035, 0.035, 7):
        for height in np.linspace(0.16, 0.26, 11):
            foot = np.array([x, -height])
            angles = leg.inverse(foot)
            points = leg.forward(angles)
            np.testing.assert_allclose(points[2], foot, atol=1e-10)
            lengths = np.linalg.norm(np.diff(np.vstack([points, points[0]]), axis=0), axis=1)
            np.testing.assert_allclose(lengths, [LEGS.upper_length, LEGS.lower_length, LEGS.lower_length,
                                               LEGS.upper_length, LEGS.hip_spacing], atol=1e-10)
            eps = 1e-6
            numerical = np.column_stack([(leg.forward(angles+np.eye(2)[i]*eps)[2]
                                        -leg.forward(angles-np.eye(2)[i]*eps)[2])/(2*eps) for i in range(2)])
            np.testing.assert_allclose(leg.jacobian(angles), numerical, atol=1e-8)
    for foot in ([0, -1], [0, 0], [np.nan, -0.2]):
        try:
            leg.inverse(foot)
        except ValueError:
            pass
        else:
            raise AssertionError('Invalid foot target accepted')
    print('PASS: 77 five-bar IK/FK closures, five link lengths, analytic Jacobian, invalid targets.')


if __name__ == '__main__':
    verify()
