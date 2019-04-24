#include "libswervedrive/estimator.h"

#include <utility>

namespace swervedrive
{
/**
 * @brief Construct a new Estimator object
 *
 * @param chassis a \sa{Chassis} object specifying the configuration of the chassis to estimate
 * @param init Starting point [x, y, theta] of the chassis in the world frame. Defaults to zero.
 * @param eta_lambda threshold when we assume the algorithm has converged to the correct
 *     icr based on how much lambda has moved between sucessive iterations
 * @param eta_delta threshold below which a starting point for iterative icr iteration is
 *     selected based on the norm between the measured wheel positions and the wheel positions obtained from that
 * starting point
 * @param min_delta_line_search minimum size of the free parameters delta_m and delta_m
 *     to avoid infinate recursion in the line search for the next iteration of the position of lambda
 * @param max_iter_lambda maximum iterations allowed for the iterative icr estimation
 *     algorithm to converge on one point
 * @param singularity_tolerance how close a point must be to be considered to be
 *     'on a structural singularity'
 */
Estimator::Estimator(const Chassis& chassis, Epsilon init, double eta_lambda, double eta_delta,
                     double min_delta_line_search, double max_iter_lambda, double singularity_tolerance)
  : chassis_(chassis)
  , epsilon_(init)
  , eta_lambda_(eta_lambda)
  , eta_delta_(eta_delta)
  , min_delta_line_search_(min_delta_line_search)
  , max_iter_lambda_(max_iter_lambda)
  , singularity_tolerance_(singularity_tolerance){};

/**
 * @brief
 *
 * @param q
 * @return Lambda
 */
Lambda Estimator::estimate(Eigen::VectorXd q)
{
  using namespace Eigen;
  using namespace std;
  using namespace swervedrive;

  std::vector<Lambda> starting_points = select_starting_points(q);

  optional<Lambda> closest_lambda = {};
  optional<double> closest_dist = {};
  for (auto lambda_start : starting_points) {

    Lambda lambda = lambda_start;
    double total_displacement = chassis_.lambda_joint_dist(q, lambda_start);

    // populate closest_lambda if not already
    if (!closest_lambda) {
      closest_lambda = lambda_start;
      closest_dist = chassis_.lambda_joint_dist(q, lambda_start);
    }

    if (chassis_.lambda_joint_dist(q, lambda) < eta_delta_) {
      return lambda;
    } else {
      for (int i=0; i < max_iter_lambda_; ++i) {
        Derivatives d = compute_derivatives(lambda);
        Deltas deltas = solve(d, q, lambda);
        bool diverging;
        Lambda lambda_t = update_parameters(lambda, deltas, q, diverging);
        optional<int> singularity = chassis_.singularity(lambda_t);

        // TODO use the singularity calc
        if (chassis_.lambda_joint_dist(q, lambda_t) > total_displacement) {
          break;
        } else if ((lambda - lambda_t).norm() < eta_lambda_) {
          return lambda_t;
        }
        lambda = lambda_t;
      }
    }
  }
  // If we exhausted all the starting points and couldn't return a value, return the closest
  // TODO closest lambda
}

/**
 * @brief Compute derivatives of constraining surface at lambda.
 *
 * @param lambda Position of ICR.
 * @return Derivatives (u, v, w), the vectors containing the derivatives of each steering angle
 *     with respect to u, v, and w respectively.
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

  auto singularity = chassis_.singularity(lambda);
  if (singularity) {
    S_m(*singularity) = 0;
    S_n(*singularity) = 0;

    gamma_bottom(*singularity) = 1;
  }

  S_m = S_m.cwiseQuotient(gamma_bottom);
  S_n = S_n.cwiseQuotient(gamma_bottom);

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

/**
 * @brief
 *
 * @param derivatives
 * @param q
 * @param lambda
 * @return Eigen::Vector3d
 */
Deltas Estimator::solve(Derivatives derivatives, Eigen::VectorXd q, Lambda lambda)
{
  using namespace Eigen;

  auto p_zero = chassis_.betas(lambda);
  auto diff = q - p_zero;
  double a_u, a_c, a_v;
  Vector2d b;
  if (!derivatives.u) {
    a_u = (*derivatives.v).dot(*derivatives.v);
    a_c = (*derivatives.v).dot(*derivatives.w);
    a_v = (*derivatives.w).dot(*derivatives.w);
    b << diff.dot(*derivatives.v), diff.dot(*derivatives.w);
  } else if (!derivatives.v) {
    a_u = (*derivatives.u).dot(*derivatives.u);
    a_c = (*derivatives.u).dot(*derivatives.w);
    a_v = (*derivatives.w).dot(*derivatives.w);
    b << diff.dot(*derivatives.u), diff.dot(*derivatives.w);
  } else {
    a_u = (*derivatives.u).dot(*derivatives.u);
    a_c = (*derivatives.u).dot(*derivatives.v);
    a_v = (*derivatives.v).dot(*derivatives.v);
    b << diff.dot(*derivatives.u), diff.dot(*derivatives.v);
  }
  MatrixXd A(2,2);
  A << a_u, a_c, a_c, a_v;
  Vector2d x = A.colPivHouseholderQr().solve(b);
  Deltas d;
  if (!derivatives.u) {
    d.u = {};
    d.v = x(0);
    d.w = x(1);
  } else if (!derivatives.v) {
    d.u = x(0);
    d.v = {};
    d.w = x(1);
  } else {
    d.u = x(0);
    d.v = x(1);
    d.w = {};
  }
  return d;
}

/**
 * @brief
 *
 * @param lambda
 * @param deltas
 * @param q
 * @param diverged
 * @return Lambda
 */
Lambda Estimator::update_parameters(Lambda lambda, Deltas deltas, Eigen::VectorXd q, bool& diverged)
{
  double m, n, delta_m, delta_n;
  std::function<Lambda(double, double)> lambda_t;
  if (!deltas.u) {
    m = lambda(1);
    n = lambda(2);
    delta_m = *deltas.v;
    delta_n = *deltas.w;
    lambda_t = [](double m, double n) {
      double residual = std::max(1 - m*m - n*n, 0.0);
      return Eigen::Vector3d(std::pow(residual, 0.5), m, n).normalized();
    };
  } else if (!deltas.v) {
    m = lambda(0);
    n = lambda(2);
    delta_m = *deltas.u;
    delta_n = *deltas.w;
    lambda_t = [](double m, double n) {
      double residual = std::max(1 - m*m - n*n, 0.0);
      return Eigen::Vector3d(m, std::pow(residual, 0.5), n).normalized();
    };
  } else {
    m = lambda(0);
    n = lambda(1);
    delta_m = *deltas.u;
    delta_n = *deltas.v;
    lambda_t = [](double m, double n) {
      double residual = std::max(1 - m*m - n*n, 0.0);
      return Eigen::Vector3d(m, n, std::pow(residual, 0.5)).normalized();
    };
  }

  double prev_m = m;
  double prev_n = n;
  double total_dist = chassis_.displacement(q, chassis_.betas(lambda)).norm();
  while (1) {
    double m_i = m + delta_m;
    double n_i = n + delta_n;
    // if adding delta_m and delta_m has produced out of bounds values,
    // recursively multiply to ensure they remain within bounds
    while (double factor = std::hypot(m_i, n_i) > 1) {
      m_i /= factor;
      n_i /= factor;
    }
    if (total_dist < chassis_.displacement(q, chassis_.betas(lambda_t(m_i, n_i))).norm()) {
      // Diverging
      // backtrack by reducing the step size
      delta_m *= 0.5;
      delta_n *= 0.5;

      // set a minimum step size to avoid infinite recursion
      if (std::hypot(delta_m, delta_n) < min_delta_line_search_) {
        // Return the previous estimate
        diverged = true;
        return lambda_t(prev_m, prev_n);
      } else {
        prev_m = m_i;
        prev_n = n_i;
      }
    } else {
      diverged = false;
      return lambda_t(m_i, n_i);
    }

  }
}

}  // namespace swervedrive
