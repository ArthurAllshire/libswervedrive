import numpy as np
import math
from .utils import constrain_angle


class Estimator:

    # constants used in the lmda estimation algo
    eta_lmda: float = 1e-4  # TODO: figure out what values this should be
    eta_delta: float = 1e-2  # TODO: figure out what values this should be
    min_delta_size: float = 1e-2  # TODO: figure out what value this should be
    max_iter = 50  # TODO: figure out what value should be
    tolerance: float = 1e-3

    def __init__(
        self,
        epsilon_init: np.ndarray,
        alpha: np.ndarray,
        l: np.ndarray,
    ):
        """
        Initialize the Estimator object. The order in the following arrays
        must be preserved throughout all arguments passed to this object.
        :param epsilon_init: the starting position estimate for the robot
            position. Form (x, y, theta)^T.
        :param alpha: array containing the angle to each of the modules,
            measured counter clockwise from the x-axis.
        :param l: distance to the axis of rotation of each module from
            the origin of the chassis frame
        """

        assert len(alpha.shape) == 2 and alpha.shape[1] == 1, alpha
        assert len(l.shape) == 2 and l.shape[1] == 1, l

        self.epsilon = epsilon_init

        self.alpha = alpha
        self.l = l
        self.n = len(self.alpha)
        self.epsilon_init = epsilon_init

        self.a = np.concatenate([np.cos(alpha).T, np.sin(alpha).T, [[0.0] * self.n]])
        self.a_orth = np.concatenate([-np.sin(alpha).T, np.cos(alpha).T, [[0.0] * self.n]])
        self.s = np.concatenate(
            [(l * np.cos(alpha)).T, (l * np.sin(alpha)).T, [[1.0] * self.n]]
        )
        self.l_v = np.concatenate([[[0.0] * self.n], [[0.0] * self.n], self.l.T])

    def estimate_lmda(self, q: np.ndarray):
        """
        Find the ICR given the steering angles.
        :param q: list of angles beta between representing the steer angle
            (measured relative to the orientation orthogonal to the line to the
            chassis frame origin.)
        :returns: our estimate of ICR as the array (u, v, w)^T.
        """
        assert len(q.shape) == 2 and q.shape[1] == 1, q
        starting_points = self.select_starting_points(q)
        found = False
        closest_lmda = None
        closest_dist = None
        for lmda_start in starting_points:
            lmda = lmda_start
            if closest_lmda is None:
                closest_lmda = lmda_start
                closest_dist = np.linalg.norm(shortest_distance(q, self.S(lmda_start)))
            if np.linalg.norm(shortest_distance(q, self.S(lmda))) < self.eta_delta:
                found = True
            else:
                last_singularity = None
                for i in range(self.max_iter):
                    (S_u, S_v, S_w) = self.compute_derivatives(lmda)
                    if last_singularity is not None:
                        # if we had a singularity last time, set the derivatives
                        # for the corresponding wheel to 0
                        if S_u is not None:
                            S_u[last_singularity] = 0
                        if S_u is not None:
                            S_v[last_singularity] = 0
                        if S_u is not None:
                            S_w[last_singularity] = 0
                    (delta_u, delta_v, delta_w) = self.solve(S_u, S_v, S_w, q, lmda)
                    lmda_t, worse = self.update_parameters(
                        lmda, delta_u, delta_v, delta_w, q
                    )
                    singularity, singularity_number = self.handle_singularities(lmda_t)
                    S_lmda = self.S(lmda_t)
                    if last_singularity is not None and singularity:
                        # the test point is still on the steering axis, suggesting
                        # it is on a singularity. Set beta_k to the input steering
                        # value
                        S_lmda[last_singularity] = q[last_singularity]
                    last_singularity = singularity_number
                    if np.linalg.norm(shortest_distance(q, S_lmda)) > np.linalg.norm(
                        shortest_distance(q, self.S(lmda_start))
                    ):
                        # appears the algorithm has diverged as we are not
                        # improving
                        found = False
                        break
                    else:
                        found = np.linalg.norm(lmda - lmda_t) < self.eta_lmda
                        distance = np.linalg.norm(shortest_distance(q, S_lmda))
                        if distance < closest_dist:
                            closest_lmda = lmda_t
                            closest_dist = distance
                    lmda = lmda_t
                    if found:
                        break
            if found:
                lmda = lmda.reshape(-1, 1)
                # Always return lamdba with a positive w component
                if lmda[2,0] < 0:
                    lmda = -lmda
                return lmda
        lmda = closest_lmda.reshape(-1, 1)
        # Always return lamdba with a positive w component
        if lmda[2,0] < 0:
            lmda = -lmda
        return lmda

    def select_starting_points(self, q: np.ndarray):
        """
        Find the starting points for the Newton-Raphson algorithm. This
        implementation places them at the intersection of the propulsion axis
        and orders them according to their distance to the input point.
        :param q: list of angles beta between representing the steer angle
            (measured relative to the orientation orthogonal to the line to the
            chassis frame origin.)
        :returns: List of the top three starting points ordered according to
        their distance to the input length.
        """
        starting_points = []
        assert len(q.shape) == 2 and q.shape[1] == 1, q

        def get_p(i):
            s = column(self.s, i).reshape(-1)
            d = np.array(
                [
                    math.cos(q[i, 0] + self.alpha[i,0]),
                    math.sin(q[i, 0] + self.alpha[i,0]),
                    0,
                ]
            )
            p = np.cross(s, d)
            p /= np.linalg.norm(p)
            return p

        for i in range(self.n):
            p_1 = get_p(i)
            for j in range(self.n):
                if not i > j:
                    continue
                p_2 = get_p(j)
                c = np.cross(p_1, p_2).reshape(-1, 1)
                if abs(p_1.dot(p_2) / np.linalg.norm(p_1) * np.linalg.norm(p_2)) > 0.99:
                    # the sine of the dot product is zero i.e. they are co-linear:
                    # Throwout cases where the two wheels being compared are co-linear
                    continue
                c /= np.linalg.norm(c)
                if c[2] < 0:
                    c = -c
                dist = np.linalg.norm(shortest_distance(q, self.S(c)))
                starting_points.append([c, dist])
        starting_points.sort(key=lambda point: point[1])
        sp_arr = [p[0].reshape(3, 1) for p in starting_points]
        return sp_arr

    def compute_derivatives(self, lmda: np.ndarray):
        """
        Compute the derivateves of the constraining surface at the current
        estimate of the point.
        :param lmda: position of the ICR estimate
        :returns: np.ndarray with (S_u, S_v, S_w). S_u, S_v, S_w are the vectors
            containing the derivatives of each steering angle in q with respect
            u, v and w respectively.
            One of them will be None because that axis is parameterised in terms
            of the other two.
        """
        assert len(lmda.shape) == 2 and lmda.shape[1] == 1, lmda

        # Work out the best hemisphere to work in
        u = lmda.T.dot(np.array([[1], [0], [0]]))
        v = lmda.T.dot(np.array([[0], [1], [0]]))
        w = lmda.T.dot(np.array([[0], [0], [1]]))
        dots = {'u': abs(u), 'v': abs(v), 'w': abs(w)}
        axis = max(dots.keys(), key=(lambda k: dots[k]))
        if axis == 'u':
            # Parameterise u
            dm = np.array([[-lmda[1,0] / lmda[0,0], 1, 0]])
            dn = np.array([[-lmda[2,0] / lmda[0,0], 0, 1]])
        elif axis == 'v':
            # Parameterise v
            dm = np.array([[1, -lmda[0,0] / lmda[1,0], 0]])
            dn = np.array([[0, -lmda[2,0] / lmda[1,0], 1]])
        else:
            # Parameterise w
            dm = np.array([[1, 0, -lmda[0,0] / lmda[2,0]]])
            dn = np.array([[0, 1, -lmda[1,0] / lmda[2,0]]])

        lmda_T = lmda.T
        lmda_T_block = np.concatenate([lmda_T]*self.n)
        diff = self.a - self.l_v
        delta = lmda_T.dot(diff)
        omega = lmda_T.dot(self.a_orth)
        gamma_top = omega * diff + delta * self.a_orth
        gamma_bottom = lmda_T.dot(delta * diff - omega * self.a_orth)
        lmda_singular = (self.s / np.linalg.norm(self.s, axis=0)).T
        is_singular = np.logical_or(
            np.all(np.isclose(lmda_T_block, lmda_singular, atol=self.tolerance), axis=1),
            np.all(np.isclose(lmda_T_block, -lmda_singular, atol=self.tolerance), axis=1)
        )
        S_m = dm.dot(gamma_top)
        S_n = dn.dot(gamma_top)
        for i,is_sing in enumerate(is_singular):
            if is_sing:
                gamma_bottom[0, i] = 1
                S_m[0, i] = 0
        S_m = (S_m / gamma_bottom).reshape(-1, 1)
        S_n = (S_n / gamma_bottom).reshape(-1, 1)

        if axis == 'u':
            return None, S_m, S_n
        if axis == 'v':
            return S_m, None, S_n
        if axis == 'w':
            return S_m, S_n, None

    def solve(
        self,
        S_u: np.ndarray,
        S_v: np.ndarray,
        S_w: np.ndarray,
        q: np.ndarray,
        lmda: np.ndarray,
    ):
        """
        Solve the system of linear equations to find the free parameters
        delta_u and delta_v.
        :param S_u: derivative of constraining surface wrt u (vector).
        :param S_v: derivative of constraining surface wrt v (vector).
        :param S_v: derivative of constraining surface wrt w (vector).
        :param q: list of angles beta representing the steer angle
        (measured relative to the orientation orthogonal to the line to the
        chassis frame origin.)
        :param lmda: position of the ICR estimate.
        :returns: the free parameters in the form (delta_u, delta_v, delta_w).
        """
        assert len(q.shape) == 2 and q.shape[1] == 1, q
        if S_u is not None:
            assert len(S_u.shape) == 2 and S_u.shape[1] == 1, S_u
        if S_v is not None:
            assert len(S_v.shape) == 2 and S_v.shape[1] == 1, S_v
        if S_w is not None:
            assert len(S_w.shape) == 2 and S_w.shape[1] == 1, S_w
        p_zero = self.S(lmda)
        diff = q - p_zero
        if S_u is None:
            a_u = S_v.T.dot(S_v)
            a_c = S_v.T.dot(S_w)
            a_v = S_w.T.dot(S_w)
            b = np.concatenate((diff.T.dot(S_v), diff.T.dot(S_w)))
        elif S_v is None:
            a_u = S_u.T.dot(S_u)
            a_c = S_u.T.dot(S_w)
            a_v = S_w.T.dot(S_w)
            b = np.concatenate((diff.T.dot(S_u), diff.T.dot(S_w)))
        else:
            a_u = S_u.T.dot(S_u)
            a_c = S_u.T.dot(S_v)
            a_v = S_v.T.dot(S_v)
            b = np.concatenate((diff.T.dot(S_u), diff.T.dot(S_v)))
        A = np.array([
            [a_u[0,0], a_c[0,0]],
            [a_c[0,0], a_v[0,0]]
        ])
        x = np.linalg.solve(A, b)
        if S_u is None:
            return None, x[0, 0], x[1, 0]
        elif S_v is None:
            return x[0, 0], None, x[1, 0]
        else:
            return x[0, 0], x[1, 0], None

    def update_parameters(
        self,
        lmda: np.ndarray,
        delta_u: float,
        delta_v: float,
        delta_w: float,
        q: np.ndarray,
    ):
        """
        Move our estimate of the ICR based on the free parameters delta_u and
        delta_v. If invalid parameters are produced rescale them so they lie
        within the sphere. If the algorithm has diverged backtrack if possible
        :param lmda: current position of the ICR estimate.
        :param delta_u: free parameter defining how much to move the ICR
            estimate in the direction S_u.
        :param delta_v: free parameter defining how much to move the ICR
            estimate in the direction S_v.
        :param delta_w: free parameter defining how much to move the ICR
            estimate in the direction S_w.
        :param q: list of angles beta representing the steer angle
            (measured relative to the orientation orthogonal to the line to the
            chassis frame origin.)
        :returns: the new ICR estimate, a flag indicating divergence of the
            algorithm for this starting point.
        """
        # Move along the non-parameterised axes. Call these m and n
        assert len(lmda.shape) == 2 and lmda.shape[1] == 1, lmda
        assert lmda.shape == (3, 1), lmda
        if delta_u is None:
            m = lmda[1, 0]
            n = lmda[2, 0]
            delta_m = delta_v
            delta_n = delta_w

            def lmda_t(m, n):
                # We normalise here so that we don't get floating point errors
                residual = max(1 - m**2 - n**2, 0.0)
                unnormed = np.array([[residual ** 0.5], [m], [n]])  # Eq 4
                return unnormed / np.linalg.norm(unnormed)

        elif delta_v is None:
            m = lmda[0, 0]
            n = lmda[2, 0]
            delta_m = delta_u
            delta_n = delta_w

            def lmda_t(m, n):
                # We normalise here so that we don't get floating point errors
                residual = max(1 - m**2 - n**2, 0.0)
                unnormed = np.array([[m], [residual ** 0.5], [n]])  # Eq 4
                return unnormed / np.linalg.norm(unnormed)

        else:
            m = lmda[0, 0]
            n = lmda[1, 0]
            delta_m = delta_u
            delta_n = delta_v

            def lmda_t(m, n):
                # We normalise here so that we don't get floating point errors
                residual = max(1 - m**2 - n**2, 0.0)
                unnormed = np.array([[m], [n], [residual ** 0.5]])  # Eq 4
                return unnormed / np.linalg.norm(unnormed)

        prev_m = m
        prev_n = n
        # while the algorithm produces a worse than or equal to good estimate
        # for q on the surface as lmda from the previous iteration
        total_dist = np.linalg.norm(shortest_distance(q, self.S(lmda)))
        while True:
            m_i = m + delta_m
            n_i = n + delta_n
            # if adding delta_m and delta_m has produced out of bounds values,
            # recursively multiply to ensure they remain within bounds
            while np.linalg.norm([m_i, n_i]) > 1:
                factor = np.linalg.norm([m_i, n_i])
                m_i /= factor
                n_i /= factor
            if total_dist < np.linalg.norm(
                shortest_distance(q, self.S(lmda_t(m_i, n_i)))
            ):
                # Diverging
                # backtrack by reducing the step size
                delta_m *= 0.5
                delta_n *= 0.5

                # set a minimum step size to avoid infinite recursion
                if np.linalg.norm([delta_m, delta_n]) < self.min_delta_size:
                    # Return the previous estimate
                    return lmda_t(prev_m, prev_n), True
                else:
                    prev_m = m_i
                    prev_n = n_i
            else:
                return lmda_t(m_i, n_i), False

    def handle_singularities(self, lmda: np.ndarray):
        """
        Handle the structural singularities that may have been produced when
        the parameters were updated (when the ICR lies on a steering axis).
        :param lmda: the ICR estimate after the parameters were updated.
        :returns: if the ICR is on a structural singularity, and the wheel
            number which the singularity is on if there is one
        """
        assert len(lmda.shape) == 2 and lmda.shape[1] == 1, lmda
        wheel_number = None
        for i in range(self.n):
            # equations 16 and 17 in the paper
            s = column(self.s, i)
            if np.allclose(lmda, s / np.linalg.norm(s)):
                wheel_number = i
                break
        return wheel_number is not None, wheel_number

    def S(self, lmda: np.ndarray):
        """
        Compute the point in the joint space (space of all beta steering angle
        values) associated with a particular ICR.
        :param lmda: the ICR to compute the point for.
        :returns: row vector expressing the point.
        """
        assert len(lmda.shape) == 2 and lmda.shape[1] == 1, lmda

        lmda_T = lmda.T
        S = np.arctan2(lmda_T.dot(self.a_orth), lmda_T.dot(self.a - self.l_v))
        S[np.isnan(S)] = math.pi / 2
        return S.reshape(-1, 1)


def shortest_distance(
    q_e: np.ndarray, q_d: np.ndarray, beta_bounds: np.ndarray = None
):
    """
    Determine if the rotation each wheel needs to make to get to the target.
    Respect the joint limits, but allow movement to a position pi from the target position
    if joint limits allow it.
    :param q_e: an array representing all of the current beta angles
    :parem q_d: an array of all the desired beta angles
    :returns: an array of the same length as the input arrays with each component
        as the correct distance of q_e to q_d.
    """
    assert len(q_e.shape) == 2 and q_e.shape[1] == 1, q_e
    assert len(q_d.shape) == 2 and q_d.shape[1] == 1, q_d

    # TODO: make this work if beta bounds is not None
    diff_opp_diff = np.concatenate([constrain_angle(q_d - q_e), constrain_angle(q_d + math.pi - q_e)], axis=1)
    choose = np.argmin(np.absolute(diff_opp_diff), axis=1).reshape(-1, 1)
    return np.take_along_axis(diff_opp_diff, choose, axis=1)


def column(mat, row_i):
    """
    Grab a column from a vector as a numpy column vector.
    :param row_i: row index
    :returns: the column vector (shape (n, 1))
    """
    return mat[:, row_i : row_i + 1]
