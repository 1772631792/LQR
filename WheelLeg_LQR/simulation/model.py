"""Force-driven inverted pendulum, theta=0 upright, positive tilt forward."""
import numpy as np
from config.robot_params import PARAMS, RobotParams


def mass_matrix(theta: float, params: RobotParams = PARAMS):
    m, l = params.body_mass, params.com_height
    return np.array([[params.base_mass + m, m * l * np.cos(theta)],
                     [m * l * np.cos(theta), params.body_inertia + m * l*l]])


def continuous_matrices(params: RobotParams = PARAMS):
    """Linearize H [p_ddot, theta_ddot]^T=[u-b*v, m*g*l*theta]^T."""
    h = mass_matrix(0.0, params)
    a = np.zeros((4, 4))
    b = np.zeros((4, 1))
    a[0, 1] = a[2, 3] = 1.0
    a[[1, 3], 1] = np.linalg.solve(h, [-params.friction, 0.0])
    a[[1, 3], 2] = np.linalg.solve(h, [0.0, params.body_mass * params.gravity * params.com_height])
    b[[1, 3], 0] = np.linalg.solve(h, [1.0, 0.0])
    return a, b


class PhysicsModel:
    def __init__(self, params: RobotParams = PARAMS, nonlinear: bool = False):
        self.params = params
        self.nonlinear = nonlinear
        self.a, self.b = continuous_matrices(params)

    def derivative(self, state, force):
        if not self.nonlinear:
            return self.a @ state + self.b[:, 0] * force
        _, velocity, theta, omega = state
        p = self.params
        rhs = [force - p.friction * velocity + p.body_mass * p.com_height * np.sin(theta) * omega**2,
               p.body_mass * p.gravity * p.com_height * np.sin(theta)]
        accel = np.linalg.solve(mass_matrix(theta, p), rhs)
        return np.array([velocity, accel[0], omega, accel[1]])

    def step(self, state, force):
        """RK4; actuator force held constant throughout one control period."""
        dt = self.params.sample_time
        f = self.derivative
        k1 = f(state, force)
        k2 = f(state + dt*k1/2, force)
        k3 = f(state + dt*k2/2, force)
        k4 = f(state + dt*k3, force)
        return state + dt*(k1 + 2*k2 + 2*k3 + k4)/6

    def impulse(self, state, impulse_ns):
        """Horizontal impulse at axle: integrate H*q_ddot across the impulse."""
        result = state.copy()
        theta = state[2] if self.nonlinear else 0.0
        result[[1, 3]] += np.linalg.solve(mass_matrix(theta, self.params), [impulse_ns, 0.0])
        return result


def verify_model():
    a, b = continuous_matrices()
    model = PhysicsModel(nonlinear=True)
    eps = 1e-6
    numerical_a = np.column_stack([(model.derivative(np.eye(4)[i]*eps, 0.0)
                                   - model.derivative(-np.eye(4)[i]*eps, 0.0))/(2*eps) for i in range(4)])
    numerical_b = (model.derivative(np.zeros(4), eps)-model.derivative(np.zeros(4), -eps))/(2*eps)
    np.testing.assert_allclose(a, numerical_a, atol=1e-8)
    np.testing.assert_allclose(b[:, 0], numerical_b, atol=1e-8)
    np.testing.assert_allclose(model.derivative(np.zeros(4), 0), 0)
    assert np.linalg.eigvalsh(mass_matrix(0)).min() > 0
    assert np.linalg.matrix_rank(np.column_stack([b, a@b, a@a@b, a@a@a@b])) == 4
    assert np.linalg.eigvals(a).real.max() > 0
    print('A =\n', a, '\nB =\n', b)
    print('Open-loop eigenvalues:', np.linalg.eigvals(a))
    print('PASS: equilibrium, positive mass matrix, finite-difference linearization, controllability.')


if __name__ == '__main__':
    verify_model()
