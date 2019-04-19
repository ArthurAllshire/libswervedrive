#include "libswervedrive/estimator.h"


namespace swervedrive {
Estimator::Estimator(Chassis chassis, Epsilon init) : chassis_(chassis), epsilon_(init) {};
Estimator::Estimator(Chassis chassis) : Estimator(chassis, Epsilon(0,0,0)) {};

/**
  Compute the point in the joint space (space of all beta steering angle
  values) associated with a particular ICR.
  :param lambda: the ICR to compute the point for.
  :returns: row vector expressing the point.
 */
Eigen::VectorXd Estimator::S(Lambda lambda) {
  auto y = (chassis_.a_orth.transpose() * lambda);
  auto x = ((chassis_.a - chassis_.l_v).transpose() * lambda);
  Eigen::VectorXd S(chassis_.n);
  for (int idx=0; idx < chassis_.n; ++idx) {
    S[idx] = atan2(y[idx], x[idx]);
    // TODO S[np.isnan(S)] = math.pi / 2
  }
  return S;
}

}
