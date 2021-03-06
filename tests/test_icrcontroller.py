import math
import numpy as np

from hypothesis import given, example, settings
from hypothesis import strategies as st
from hypothesis.extra.numpy import arrays

from .helpers import unlimited_rotation_controller, build_controller, twist_to_icr

from swervedrive.icr.estimator import shortest_distance


def cartesian_to_lambda(x, y):
    """Convert an ICR in R^2 to one in the projective plane."""
    return np.reshape(1 / np.linalg.norm([x, y, 1]) * np.array([x, y, 1]), (3, 1))


def test_icrc_init():
    c = unlimited_rotation_controller(
        [-0.5, 0.5], [-1e-6, 1e-6], [-1e-6, 1e-6], [-1e-6, 1e-6]
    )
    assert c is not None


def assert_velocity_bounds(c, delta_beta, phi_dot_cmd, dt):
    # Check limits are respected
    tol = 1e-3  # to ensure we don't go over due to a floating point error
    assert all(
        [(db) >= (c.beta_dot_bounds[0] * dt) - tol for db in delta_beta]
    ), "Bounds: %s\nDelta beta: %s" % (c.beta_dot_bounds * dt, delta_beta)
    assert all(
        [(db) <= (c.beta_dot_bounds[1] * dt) + tol for db in delta_beta]
    ), "Bounds: %s\nDelta beta: %s" % (c.beta_dot_bounds * dt, delta_beta)
    assert all(
        (pc) >= (c.phi_dot_bounds[0]) - tol for pc in phi_dot_cmd
    ), "Bounds: %s\nDelta beta: %s" % (c.phi_dot_bounds * dt, phi_dot_cmd)
    assert all(
        (pc) <= (c.phi_dot_bounds[1]) + tol for pc in phi_dot_cmd
    ), "Bounds: %s\nDelta beta: %s" % (c.phi_dot_bounds * dt, phi_dot_cmd)


@given(
    lmda_d=arrays(np.float, (1, 3), elements=st.floats(min_value=1e-6, max_value=1)),
    lmda_d_sign=st.floats(
        min_value=-1, max_value=1
    ),  # make this *float* to give uniform distribution
    bounds=st.lists(st.floats(1e-1, 10), min_size=4, max_size=4),
)
@example(
    lmda_d=np.array([[1.00000000e-06, 4.9624216e-06, 4.9624216e-06]]),
    lmda_d_sign=0.0,
    bounds=[0.1, 0.1, 0.1, 0.1],
)
@settings(max_examples=100, deadline=1000)
def test_converge_respect_velocity_bounds(lmda_d, lmda_d_sign, bounds):
    from swervedrive.icr.kinematicmodel import KinematicModel

    # could do this using st.builds but that does not provide a view into what
    # the values that caused the failure were which is useful for diagnosis
    c = build_controller([-b for b in bounds], bounds)  # Symmetric bounds

    # Modules can only rotate at a maximum of 0.5 rad/s
    # Make sure the controller respects these limits
    iterations = 0
    modules_beta = np.array([[0]] * 4)
    modules_phi_dot = np.array([[0]] * 4)
    lmda_d = (math.copysign(1, lmda_d_sign) * lmda_d / np.linalg.norm(lmda_d)).reshape(
        -1, 1
    )
    q_d = c.icre.S(lmda_d)

    mu_d = bounds[2] * c.r[0, 0] / c.l[0, 0] * 0.1  # 0.1 of max spot rotation
    dt = 0.1

    beta_prev = modules_beta
    delta_beta = np.array([[1]] * 4)
    phi_dot_prev = modules_phi_dot
    beta_history = []
    lmda_history = []
    mu_e = 0
    lmda_e = c.icre.estimate_lmda(beta_prev)

    stop_iter = 200

    while iterations < stop_iter and not (
        np.isclose(shortest_distance(beta_prev, q_d), 0, atol=math.pi * 1 / 180).all()
        and (np.isclose(mu_e, mu_d, atol=1e-2) or np.isclose(-mu_e, mu_d, atol=1e-2))
    ):
        beta_cmd, phi_dot_cmd, xi_e = c.control_step(
            beta_prev, phi_dot_prev, lmda_d, mu_d, dt
        )
        assert c.kinematic_model.state == KinematicModel.State.RUNNING

        delta_beta = shortest_distance(beta_cmd, beta_prev)
        assert_velocity_bounds(c, delta_beta, phi_dot_cmd, dt)
        beta_history.append(beta_cmd)
        # lmda_history.append(c.icre.estimate_lmda(beta_cmd))

        beta_prev = beta_cmd
        phi_dot_prev = phi_dot_cmd
        lmda_e = c.icre.estimate_lmda(beta_prev)
        mu_e = c.kinematic_model.estimate_mu(phi_dot_prev, lmda_e)
        iterations += 1

    lmda_e = c.icre.estimate_lmda(beta_prev)
    mu_e = c.kinematic_model.estimate_mu(phi_dot_prev, lmda_e)
    assert np.isclose(
        shortest_distance(beta_prev, q_d), 0, atol=math.pi * 5 / 180
    ).all(), (
        "Controller did not reach target:\n%s\nactual: %s\nbeta history: %s\nlambda history: %s"
        % (lmda_d, lmda_e, beta_history, lmda_history)
    )
    assert np.isclose(mu_e, mu_d, atol=1e-1) or np.isclose(-mu_e, mu_d, atol=1e-1)


def test_structural_singularity_command():
    c = unlimited_rotation_controller(
        [-0.5, 0.5], [-1e-6, 1e-6], [-1e-6, 1e-6], [-1e-6, 1e-6]
    )
    # test to see what happens if we place the ICR on a structural singularity
    iterations = 0
    modules_beta = np.array([[0]] * 4)
    modules_phi_dot = np.array([[0]] * 4)
    lmda_d_normal = np.array([[1], [0], [0]])

    lmda_singularity = cartesian_to_lambda(
        c.l[0, 0] * math.sin(c.alpha[0, 0]), c.l[1, 0] * math.sin(c.alpha[1, 0])
    )

    mu_d = 0.1
    dt = 0.1

    beta_prev = modules_beta
    phi_dot_prev = modules_phi_dot
    while iterations < 100:
        # let the controller do its thing for a while, then put the command on a singularity
        lmda_d = lmda_d_normal
        if iterations > 20:
            lmda_d = lmda_singularity

        beta_cmd, phi_dot_cmd, xi_e = c.control_step(
            beta_prev, phi_dot_prev, lmda_d, mu_d, dt
        )

        # check that our estimated ICR never gets close to the structural singularity,
        # despite being the setpoint
        lmda_e = c.icre.estimate_lmda(beta_cmd).reshape(3)
        assert not np.allclose(lmda_e, lmda_singularity, atol=1e-2)

        beta_prev = beta_cmd
        phi_dot_prev = phi_dot_cmd
        iterations += 1


def test_bad_s_2dot():
    c = unlimited_rotation_controller([-5, 5], [-20, 20], [-128, 128], [-100, 100])

    modules_beta = np.array([[0], [math.pi / 2], [0], [math.pi / 2]])
    modules_phi_dot = np.array([[0]] * 4)

    lmda_d, mu_d = twist_to_icr(0, 1, 0)
    dt = 1 / 10
    beta_prev = modules_beta
    phi_dot_prev = modules_phi_dot
    iterations = 0
    while iterations < 100:
        beta_cmd, phi_dot_cmd, xi_e = c.control_step(
            beta_prev, phi_dot_prev, lmda_d, mu_d, dt
        )
        beta_prev = beta_cmd
        phi_dot_prev = phi_dot_cmd
        iterations += 1


@given(
    lmda_initial=arrays(
        np.float, (1, 3), elements=st.floats(min_value=1e-6, max_value=1)
    ),
    lmda_initial_sign=st.floats(
        min_value=-1, max_value=1
    ),  # make this *float* to give uniform distribution
    lmda_goal=arrays(np.float, (1, 3), elements=st.floats(min_value=1e-6, max_value=1)),
    lmda_goal_sign=st.floats(
        min_value=-1, max_value=1
    ),  # make this *float* to give uniform distribution
)
def test_find_path(lmda_initial, lmda_initial_sign, lmda_goal, lmda_goal_sign):
    mu_d = -1.0

    lmda_initial = (
        math.copysign(1, lmda_initial_sign)
        * lmda_initial
        / np.linalg.norm(lmda_initial)
    ).reshape(-1, 1)
    lmda_goal = (
        math.copysign(1, lmda_goal_sign) * lmda_goal / np.linalg.norm(lmda_goal)
    ).reshape(-1, 1)
    c = unlimited_rotation_controller([-5, 5], [-20, 20], [-128, 128], [-100, 100])
    beta_prev = c.icre.S(lmda_initial)
    q_d = c.icre.S(lmda_goal)
    phi_dot_prev = np.array([[0]] * 4)
    mu_e = 0
    dt = 0.1

    iterations = 0
    beta_history = []
    lmda_history = []
    mu_history = []

    stop_iter = 50

    while iterations < stop_iter and not (
        np.isclose(shortest_distance(beta_prev, q_d), 0, atol=math.pi * 1 / 180).all()
        and (np.isclose(mu_e, mu_d, atol=1e-1) or np.isclose(-mu_e, mu_d, atol=1e-1))
    ):
        beta_cmd, phi_dot_cmd, xi_e = c.control_step(
            beta_prev, phi_dot_prev, lmda_goal, mu_d, dt
        )
        beta_prev = beta_cmd
        phi_dot_prev = phi_dot_cmd
        lmda_e = c.icre.estimate_lmda(beta_prev)
        mu_e = c.kinematic_model.estimate_mu(phi_dot_prev, lmda_e)
        beta_history.append(beta_cmd)
        mu_history.append(mu_e)
        iterations += 1
    assert iterations < stop_iter, (
        "Controller did not reach target:\n%s\nactual: %s\n \
        beta target: %s\nbeta history: %s\nlambda history: %s\nmu target: %s\nmu actual: %s"
        % (lmda_goal, lmda_e, q_d, beta_history, lmda_history, mu_d, mu_history)
    )
