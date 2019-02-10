import numpy as np
import math


# constants used in the lmda estimation algo
eta_lmda = 1e-4  # TODO: figure out what values this should be
eta_delta = 1e-2  # TODO: figure out what values this should be
min_delta_size = 1e-2  # TODO: figure out what value this should be
max_iter = 50  # TODO: figure out what value should be
tolerance = 1e-3


def make_vectors(alpha, l):
    """
    Form the vectors required to efficiently run the estimator
    :param alpha: array containing the angle to each of the modules,
        measured counter clockwise from the x-axis.
    :param l: distance to the axis of rotation of each module from
        the origin of the chassis frame
    :returns: a, a_orth, s, l_v: vectors required to run the estimator
    """
    assert len(alpha.shape) == 2 and alpha.shape[1] == 1, str(alpha)
    assert len(l.shape) == 2 and l.shape[1] == 1, str(l)

    n = alpha.shape[0]

    alpha_row = alpha.reshape(-1)
    one_row = np.ones(alpha_row.shape)
    l_row = l.reshape(-1)

    cos = np.cos(alpha_row)
    sin = np.sin(alpha_row)

    a = np.zeros((3, n))
    a[0] = cos
    a[1] = sin
    a_orth = np.zeros((3, n))
    a_orth[0] = -sin
    a_orth[1] = cos
    s = np.zeros((3, n))
    s[0] = l_row * cos
    s[1] = l_row * sin
    s[2] = one_row
    l_v = np.zeros((3, n))
    l_v[2] = l_row

    return a, a_orth, s, l_v


def estimate_lmda(q, alpha, a, a_orth, s, l_v):
    """
    Find the ICR given the steering angles.
    :param q: list of angles beta between representing the steer angle
        (measured relative to the orientation orthogonal to the line to the
        chassis frame origin.)
    :param a: return value from make_vectors
    :param a_orth: return value from make_vectors
    :param s: return value from make_vectors
    :param l_v: return value from make_vectors
    :returns: our estimate of ICR as the array (u, v, w)^T.
    """
    assert len(alpha.shape) == 2 and alpha.shape[1] == 1, str(alpha)
    assert len(q.shape) == 2 and q.shape[1] == 1, str(q)
    n = q.shape[0]
    starting_points = select_starting_points(q, alpha, a, a_orth, s, l_v, n)
    return starting_points[0]
    found = False
    # not initialzed yet, but have these here for scoping
    closest_lmda = np.zeros(shape=(3, 1))
    closest_lmda_init = False
    closest_dist = -1.
    for lmda_start in starting_points:
        lmda = lmda_start
        if not closest_lmda_init:
            closest_lmda = lmda_start
            closest_lmda_init = True
            closest_dist = np.linalg.norm(shortest_distance(q, S(lmda_start, a, a_orth, l_v)))
        if np.linalg.norm(shortest_distance(q, S(lmda, a, a_orth, l_v))) < eta_delta:
            found = True
        else:
            last_singularity = -1
            for i in range(max_iter):
                S_u, S_v, S_w, none_axis = compute_derivatives(lmda, a, a_orth, s, l_v, n)
                if last_singularity is not -1:
                    # if we had a singularity last time, set the derivatives
                    # for the corresponding wheel to 0
                    if S_u is not None:
                        S_u[last_singularity] = 0
                    if S_u is not None:
                        S_v[last_singularity] = 0
                    if S_u is not None:
                        S_w[last_singularity] = 0
                (delta_u, delta_v, delta_w) = solve(S_u, S_v, S_w, none_axis, q, lmda, a, a_orth, l_v, n)
                lmda_t, worse = update_parameters(
                    lmda, delta_u, delta_v, delta_w, none_axis, q, a, a_orth, l_v, n
                )
                singularity, singularity_number = handle_singularities(lmda_t, s, n)
                S_lmda = S(lmda_t, a, a_orth, l_v)
                if last_singularity is not -1 and singularity:
                    # the test point is still on the steering axis, suggesting
                    # it is on a singularity. Set beta_k to the input steering
                    # value
                    S_lmda[last_singularity] = q[last_singularity]
                last_singularity = singularity_number
                if np.linalg.norm(shortest_distance(q, S_lmda)) > np.linalg.norm(
                    shortest_distance(q, S(lmda_start, a, a_orth, l_v))
                ):
                    # appears the algorithm has diverged as we are not
                    # improving
                    found = False
                    break
                else:
                    found = np.linalg.norm(lmda - lmda_t) < eta_lmda
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


def select_starting_points(q, alpha,
                           a, a_orth,
                           s, l_v, n):
    """
    Find the starting points for the Newton-Raphson algorithm. This
    implementation places them at the intersection of the propulsion axis
    and orders them according to their distance to the input point.
    :param q: list of angles beta between representing the steer angle
        (measured relative to the orientation orthogonal to the line to the
        chassis frame origin.)
    :param alpha: array containing the angle to each of the modules,
        measured counter clockwise from the x-axis.
    :param a: return value from make_vectors
    :param a_orth: return value from make_vectors
    :param s: return value from make_vectors
    :param l_v: return value from make_vectors
    :param n: number of wheels
    :returns: List of the top three starting points ordered according to
    their distance to the input length.
    """
    starting_points = []
    assert len(q.shape) == 2 and q.shape[1] == 1, str(q)

    def get_p(i):
        s_col = column(s, i).reshape(-1)
        d = np.array(
            [
                math.cos(q[i, 0] + alpha[i,0]),
                math.sin(q[i, 0] + alpha[i,0]),
                0,
            ]
        )
        p = cross(s_col, d)
        p /= np.linalg.norm(p)
        return p

    cs = []
    dists = []

    for i in range(n):
        p_1 = get_p(i)
        for j in range(n):
            if not i > j:
                continue
            p_2 = get_p(j)
            c = cross(p_1, p_2).reshape(-1, 1)
            if abs(p_1.dot(p_2) / np.linalg.norm(p_1) * np.linalg.norm(p_2)) > 0.99:
                # the sine of the dot product is zero i.e. they are co-linear:
                # Throwout cases where the two wheels being compared are co-linear
                continue
            c /= np.linalg.norm(c)
            if c[2] < 0:
                c = -c
            dist = np.linalg.norm(shortest_distance(q, S(c, a, a_orth, l_v)))
            cs.append(c)
            dists.append(dist)   
    # can't use key-sorting so have to manually sort
    starting_points = []
    for i in range(len(dists)):
        smallest_arg = np.argmin(dists)
        starting_points.append(cs.pop(smallest_arg).reshape(3, 1))
        dists.pop(smallest_arg)
    return starting_points


def compute_derivatives(lmda, a, a_orth, s,
                        l_v, n):
    """
    Compute the derivateves of the constraining surface at the current
    estimate of the point.
    :param lmda: position of the ICR estimate
    :param a: return value from make_vectors
    :param a_orth: return value from make_vectors
    :param s: return value from make_vectors
    :param l_v: return value from make_vectors
    :param n: number of wheels
    :returns with (S_u, S_v, S_w). S_u, S_v, S_w are the vectors
        containing the derivatives of each steering angle in q with respect
        u, v and w respectively.
        One of them will be None because that axis is parameterised in terms
        of the other two.
    """
    assert len(lmda.shape) == 2 and lmda.shape[1] == 1, str(lmda)

    # Work out the best hemisphere to work in
    axes = ['u', 'v', 'w']
    axis_idx = np.argmax(lmda.reshape(-1))
    axis = axes[axis_idx]

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
    lmda_T_block = np.concatenate([lmda_T]*n)
    diff = a - l_v
    delta = lmda_T.dot(diff)
    omega = lmda_T.dot(a_orth)
    gamma_top = omega * diff + delta * a_orth
    gamma_bottom = lmda_T.dot(delta * diff - omega * a_orth)
    lmda_singular = (s / np.linalg.norm(s, axis=0)).T
    is_singular = np.logical_or(
        np.all(np.isclose(lmda_T_block, lmda_singular, atol=tolerance), axis=1),
        np.all(np.isclose(lmda_T_block, -lmda_singular, atol=tolerance), axis=1)
    )
    S_m = dm.dot(gamma_top)
    S_n = dn.dot(gamma_top)
    for i,is_sing in enumerate(is_singular):
        if is_sing:
            gamma_bottom[0, i] = 1
            S_m[0, i] = 0
    S_m = (S_m / gamma_bottom).reshape(-1, 1)
    S_n = (S_n / gamma_bottom).reshape(-1, 1)

    zeros = np.zeros(S_m.shape)

    if axis == 'u':
        return zeros, S_m, S_n, axis
    if axis == 'v':
        return S_m, zeros, S_n, axis
    if axis == 'w':
        return S_m, S_n, zeros, axis


def solve(
    S_u,
    S_v,
    S_w,
    none_axis,
    q,
    lmda,
    a,
    a_orth,
    l_v,
    n
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
    :param a: return value from make_vectors
    :param a_orth: return value from make_vectors
    :param l_v: return value from make_vectors
    :param n: number of wheels
    :returns: the free parameters in the form (delta_u, delta_v, delta_w).
    """
    assert len(q.shape) == 2 and q.shape[1] == 1, str(q)
    # if S_u is not None:
    #     assert len(S_u.shape) == 2 and S_u.shape[1] == 1, str(S_u)
    # if S_v is not None:
    #     assert len(S_v.shape) == 2 and S_v.shape[1] == 1, str(S_v)
    # if S_w is not None:
    #     assert len(S_w.shape) == 2 and S_w.shape[1] == 1, str(S_w)
    p_zero = S(lmda, a, a_orth, l_v)
    diff = q - p_zero
    a_u = np.zeros((1,1))
    a_c = np.zeros((1,1))
    a_v = np.zeros((1,1))
    b = np.zeros((2,1))
    if none_axis == "u":
        a_u = S_v.T.dot(S_v)
        a_c = S_v.T.dot(S_w)
        a_v = S_w.T.dot(S_w)
        b = np.concatenate((diff.T.dot(S_v), diff.T.dot(S_w)))
    elif none_axis == "v":
        a_u = S_u.T.dot(S_u)
        a_c = S_u.T.dot(S_w)
        a_v = S_w.T.dot(S_w)
        b = np.concatenate((diff.T.dot(S_u), diff.T.dot(S_w)))
    else: # none axis is "w"
        a_u = S_u.T.dot(S_u)
        a_c = S_u.T.dot(S_v)
        a_v = S_v.T.dot(S_v)
        b = np.concatenate((diff.T.dot(S_u), diff.T.dot(S_v)))
    A = np.array([
        [a_u[0,0], a_c[0,0]],
        [a_c[0,0], a_v[0,0]]
    ])
    A_inv = getTwoSquareMatrixInverse(A)
    x = A_inv.dot(b)
    if none_axis == "u":
        return None, x[0, 0], x[1, 0]
    elif none_axis == "v":
        return x[0, 0], None, x[1, 0]
    else:
        return x[0, 0], x[1, 0], None


def update_parameters(
    lmda,
    delta_u,
    delta_v,
    delta_w,
    none_axis,
    q,
    a,
    a_orth,
    l_v,
    n
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
    :param a: return value from make_vectors
    :param a_orth: return value from make_vectors
    :param l_v: return value from make_vectors
    :param n: number of wheels
    :returns: the new ICR estimate, a flag indicating divergence of the
        algorithm for this starting point.
    """
    # Move along the non-parameterised axes. Call these m and n
    assert len(lmda.shape) == 2 and lmda.shape[1] == 1, str(lmda)
    assert lmda.shape == (3, 1), str(lmda)
    if none_axis == "u":
        m = lmda[1, 0]
        n = lmda[2, 0]
        delta_m = delta_v
        delta_n = delta_w

        def lmda_t(m, n):
            # We normalise here so that we don't get floating point errors
            residual = max(1 - m**2 - n**2, 0.0)
            unnormed = np.array([[residual ** 0.5], [m], [n]])  # Eq 4
            return unnormed / np.linalg.norm(unnormed)

    elif none_axis == "v":
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
    total_dist = np.linalg.norm(shortest_distance(q, S(lmda, a, a_orth, l_v)))
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
            shortest_distance(q, S(lmda_t(m_i, n_i), a, a_orth, l_v))
        ):
            # Diverging
            # backtrack by reducing the step size
            delta_m *= 0.5
            delta_n *= 0.5

            # set a minimum step size to avoid infinite recursion
            if np.linalg.norm([delta_m, delta_n]) < min_delta_size:
                # Return the previous estimate
                return lmda_t(prev_m, prev_n), True
            else:
                prev_m = m_i
                prev_n = n_i
        else:
            return lmda_t(m_i, n_i), False


def handle_singularities(lmda, s, n):
    """
    Handle the structural singularities that may have been produced when
    the parameters were updated (when the ICR lies on a steering axis).
    :param lmda: the ICR estimate after the parameters were updated.
    :param s: return value from make_vectors
    :param n: number of wheels
    :returns: if the ICR is on a structural singularity, and the wheel
        number which the singularity is on if there is one
    """
    assert len(lmda.shape) == 2 and lmda.shape[1] == 1, str(lmda)
    wheel_number = None
    for i in range(n):
        # equations 16 and 17 in the paper
        s_col = column(s, i)
        if np.allclose(lmda, s_col / np.linalg.norm(s_col)):
            wheel_number = i
            break
    return wheel_number is not None, wheel_number


def S(lmda,
      a,
      a_orth,
      l_v,
      ):
    """
    Compute the point in the joint space (space of all beta steering angle
    values) associated with a particular ICR.
    :param lmda: the ICR to compute the point for.
    :param a: return value from make_vectors
    :param a_orth: return value from make_vectors
    :param l_v: return value from make_vectors
    :param n: number of wheels
    :returns: row vector expressing the point.
    """
    assert len(lmda.shape) == 2 and lmda.shape[1] == 1, str(lmda)

    lmda_T = lmda.T
    S = np.arctan2(lmda_T.dot(a_orth), lmda_T.dot(a - l_v))
    S[np.isnan(S)] = math.pi / 2
    return S.reshape(-1, 1)


def shortest_distance(
    q_e, q_d
    # TODO: re-add support for beta_bounds here, it broke pythran
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
    assert len(q_e.shape) == 2 and q_e.shape[1] == 1, str(q_e)
    assert len(q_d.shape) == 2 and q_d.shape[1] == 1, str(q_d)

    # TODO: make this work if beta bounds is not None
    diff_opp_diff = np.hstack((constrain_angle(q_d - q_e), constrain_angle(q_d + math.pi - q_e)))
    choose = np.argmin(np.absolute(diff_opp_diff), axis=1).reshape(-1)
    taken = np.zeros(shape=q_e.shape)
    for i, argmin in enumerate(choose):
        taken[i,0] = diff_opp_diff[i, argmin]
    return taken


def column(mat, row_i):
    """
    Grab a column from a vector as a numpy column vector.
    :param row_i: row index
    :returns: the column vector (shape (n, 1))
    """
    return mat[:, row_i : row_i + 1]


def cross(a, b):
    """
    Simple cross product implementation
    """
    return np.array([a[1]*b[2] - a[2]*b[1],
                     a[2]*b[0] - a[0]*b[2],
                     a[0]*b[1] - a[1]*b[0]])


def getTwoSquareMatrixInverse(m):
    determinant = m[0][0]*m[1][1]-m[0][1]*m[1][0]
    return np.array([[m[1][1]/determinant, -1*m[0][1]/determinant],
                    [-1*m[1][0]/determinant, m[0][0]/determinant]])


def constrain_angle(angle):
    """ Wrap `angle` to +- pi"""
    return np.arctan2(np.sin(angle), np.cos(angle))
