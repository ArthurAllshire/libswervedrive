import cProfile
import math
import numpy as np

from swervedrive.icr.estimator_pythran import make_vectors, estimate_lmda, S


if __name__ == "__main__":
    alpha = np.array([0, math.pi/2, math.pi, 3*math.pi/2]).reshape(-1, 1)
    l = np.ones((4, 1))
    a, a_orth, s, l_v = make_vectors(alpha, l)

    lmda_d = np.array([0.5, 0.5, 0.1]).reshape(-1, 1)
    lmda_d /= np.linalg.norm(lmda_d)
    print(f'lmda_actual {lmda_d}')
    # errors = np.random.randn(4, 1) * 0.1
    errors = np.array([0.2, 0.06, 0.1, -0.03]).reshape(-1, 1)
    modules_beta = S(lmda_d, a, a_orth, l_v) + errors

    cProfile.run('estimate_lmda(modules_beta, alpha, a, a_orth, s, l_v)')
    print(estimate_lmda(modules_beta, alpha, a, a_orth, s, l_v))
    print(f"errors {errors} modules_beta {modules_beta}")
