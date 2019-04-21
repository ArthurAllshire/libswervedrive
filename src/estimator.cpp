#include "libswervedrive/estimator.h"

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

Derivatives Estimator::compute_derivatives(Lambda lambda)
{
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
