#include "libswervedrive/estimator.h"

#include <utility>

namespace swervedrive
{
Estimator::Estimator(const Chassis& chassis, Epsilon init, double eta_lambda, double eta_delta,
                     double min_delta_line_search, double max_iter_lambda, double singularity_tolerance)
  : chassis_(chassis)
  , epsilon_(init)
  , eta_lambda_(eta_lambda)
  , eta_delta_(eta_delta)
  , min_delta_line_search_(min_delta_line_search)
  , max_iter_lambda_(max_iter_lambda)
  , singularity_tolerance_(singularity_tolerance){};

Lambda Estimator::estimate(Eigen::VectorXd q)
{
}

/**
  Compute the derivatives of the constraining surface at the current
  estimate of the point.
  :param lmda: position of the ICR estimate
  :returns: np.ndarray with (S_u, S_v, S_w). S_u, S_v, S_w are the vectors
  containing the derivatives of each steering angle in q with respect
  u, v and w respectively.
  One of them will be None because that axis is parameterised in terms
  of the other two.
 */
Derivatives Estimator::compute_derivatives(Lambda lambda)
{
  using namespace Eigen;
  using namespace std;
  using namespace swervedrive;
  // Work out the best hemisphere to work in
  RowVector3d dm, dn;
  vector<pair<char, double>> dots;
  dots.push_back(make_pair('u', abs(lambda.dot(Vector3d(1, 0, 0)))));
  dots.push_back(make_pair('v', abs(lambda.dot(Vector3d(0, 1, 0)))));
  dots.push_back(make_pair('w', abs(lambda.dot(Vector3d(0, 0, 1)))));
  sort(dots.begin(), dots.end(),
       [](const pair<char, double> p1, const pair<char, double> p2) { return p1.second > p2.second; });
  char axis = dots[0].first;
  if (axis == 'u')
  {
    dm << -lambda(1) / lambda(0), 1, 0;
    dn << -lambda(2) / lambda(0), 0, 1;
  }
  else if (axis == 'v')
  {
    dm << 1, -lambda(0) / lambda(0), 0;
    dn << 0, -lambda(2) / lambda(0), 1;
  }
  else
  {
    dm << 1, 0, -lambda(0) / lambda(0);
    dn << 0, 1, -lambda(1) / lambda(0);
  }

  // we need to elementwise-multiply through broadcasting delta and omega
  // (which are both 1xn row vectors) by three-row matrices, so create
  Eigen::MatrixXd delta_block(3, chassis_.n_);
  Eigen::MatrixXd omega_block(3, chassis_.n_);

  auto diff = chassis_.a_ - chassis_.l_;
  auto delta = lambda.transpose() * diff;
  delta_block << delta, delta, delta;
  auto omega = lambda.transpose() * chassis_.a_orth_;
  omega_block << omega, omega, omega;


  // 3 x n
  MatrixXd gamma_top = (MatrixXd) omega_block.cwiseProduct(diff) + delta_block.cwiseProduct(chassis_.a_orth_);
  // 1 x n
  RowVectorXd gamma_bottom = (RowVectorXd)
      lambda.transpose() * (delta_block.cwiseProduct(diff) - omega_block.cwiseProduct(chassis_.a_orth_));


  RowVectorXd S_m = (dm * gamma_top);
  RowVectorXd S_n = (dn * gamma_top);

  for (int idx = 0; idx < chassis_.n_; ++idx)
  {
    auto s_norm = chassis_.s_.col(idx).normalized();
    if (lambda.isApprox(s_norm) || lambda.isApprox(-s_norm))
    {
      S_m(idx) = 0;
      // TODO: Do we need this line here:
      S_n(idx) = 0;

      gamma_bottom(idx) = 1;
    }
  }

  S_m = S_m.cwiseQuotient(gamma_bottom);
  S_n = S_n.cwiseQuotient(gamma_bottom);

  /*
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
*/
  Derivatives derivatives;
  if (axis == 'u')
  {
    derivatives.u = {};
    derivatives.v = S_m;
    derivatives.w = S_n;
  }
  else if (axis == 'v')
  {
    derivatives.u = S_m;
    derivatives.v = {};
    derivatives.w = S_n;
  }
  else
  {
    derivatives.u = S_m;
    derivatives.v = S_n;
    derivatives.w = {};
  }
  return derivatives;
}

/**
  @brief
  Find the starting points for the Newton-Raphson algorithm. This
  implementation places them at the intersection of the propulsion axis
  and orders them according to their distance to the input point.
  @param q list of angles beta between representing the steer angle
  (measured relative to the orientation orthogonal to the line to the
  chassis frame origin.)
  @returns List of the top three starting points ordered according to
  their distance to the input length.
 */
std::vector<Lambda> Estimator::select_starting_points(Eigen::VectorXd q)
{
  using namespace Eigen;
  using namespace std;
  using namespace swervedrive;

  auto angle_sum = q + chassis_.alpha_;
  auto d = MatrixXd(chassis_.n_, 3);
  d << cos(angle_sum.array()), sin(angle_sum.array()), VectorXd(chassis_.n_);

  vector<Vector3d> p;
  for (int idx = 0; idx < chassis_.n_; ++idx)
  {
    Vector3d si = chassis_.s_.col(idx);
    Vector3d di = d.row(idx);
    auto p_i = si.cross(di).normalized();
    p.push_back(p_i);
  }

  vector<Lambda> starting_points;
  for (int i = 0; i < chassis_.n_; ++i)
  {
    auto p_i = p[i];
    for (int j = i + 1; j < chassis_.n_; ++j)
    {
      auto p_j = p[j];
      if (p_i.isApprox(p_j) || p_i.isApprox(-p_j))
      {
        // Throw away collinear axes
        continue;
      }
      auto c = p_i.cross(p_j).normalized();
      if (c(2) < 0)
      {
        c = -c;
      }
      starting_points.push_back(c);
    }
  }
  // Sort by distance from current configuration
  std::sort(starting_points.begin(), starting_points.end(), [this, q](const Vector3d lambda1, const Vector3d lambda2) {
    auto q1 = chassis_.betas(lambda1);
    auto q2 = chassis_.betas(lambda2);
    return chassis_.displacement(q, q1).array().abs().sum() < chassis_.displacement(q, q2).array().abs().sum();
  });
  return starting_points;
}

Eigen::Vector3d Estimator::solve(Derivatives derivatives, Eigen::VectorXd q, Lambda lambda)
{
}
Lambda Estimator::update_parameters(Lambda lambda, Eigen::Vector3d deltas, Eigen::VectorXd q)
{
}

}  // namespace swervedrive
