#include "libswervedrive/chassis.h"

using namespace Eigen;

namespace swervedrive
{
/**
 * @brief Construct a new Chassis object
 *
 * @param alpha Array containing the angle to each of the modules measured counter clockwise from the x-axis in
 * radians
 * @param l Distance from the centre of the robot to each module's steering axis, in m
 * @param r Radii of the wheels, in m
 * @param b Horizontal distance from the axis of rotation of each module to its contact with the gound, in m
 */
Chassis::Chassis(VectorXd alpha, VectorXd l, VectorXd r, VectorXd b) : alpha_(alpha), l_vector_(l), r_(r), b_vector_(b)
{
  n_ = alpha.size();
  a_ = MatrixXd(3, n_);
  a_ << cos(alpha_.array()).transpose(), sin(alpha_.array()).transpose(), VectorXd::Zero(n_).transpose();
  a_orth_ = MatrixXd(3, n_);
  a_orth_ << -sin(alpha_.array()).transpose(), cos(alpha_.array()).transpose(), VectorXd::Zero(n_).transpose();
  s_ = MatrixXd(3, n_);
  s_ << (cos(alpha_.array()) * l_vector_.array()).transpose(), (sin(alpha_.array()) * l_vector_.array()).transpose(),
      VectorXd::Constant(n_, 1).transpose();

  l_ = MatrixXd(3, n_);
  l_ << MatrixXd::Zero(2, n_), l_vector_.transpose();
  b_ = MatrixXd(3, n_);
  b_ << MatrixXd::Zero(2, n_), b_vector_.transpose();

  // Set unlimited bounds by default
  Vector2d unlimited = { -1.0e10, 1.0e10 };
  beta_bounds_ = Bounds(n_, unlimited);
  beta_dot_bounds_ = Bounds(n_, unlimited);
  beta_2dot_bounds_ = Bounds(n_, unlimited);
  phi_dot_bounds_ = Bounds(n_, unlimited);
  phi_2dot_bounds_ = Bounds(n_, unlimited);
}

/**
 * @brief Construct a new Chassis object
 *
 * @param alpha Array containing the angle to each of the modules measured counter clockwise from the x-axis in
 * radians
 * @param l Distance from the centre of the robot to each module's steering axis, in m
 * @param r Radii of the wheels, in m
 */
Chassis::Chassis(VectorXd alpha, VectorXd l, VectorXd r) : Chassis(alpha, l, r, VectorXd::Zero(alpha.size()))
{
}

/**
 * @param beta_bounds Min/max allowable value for steering angle, in rad.
 */
bool Chassis::setBetaBounds(Bounds bounds)
{
  if (bounds.size() == n_)
  {
    beta_bounds_ = bounds;
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @param beta_dot_bounds Min/max allowable value for rotation rate of modules, in rad/s
 */
bool Chassis::setBetaDotBounds(Bounds bounds)
{
  if (bounds.size() == n_)
  {
    beta_dot_bounds_ = bounds;
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @param beta_2dot_bounds Min/max allowable value for the angular acceleration of the modules, in rad/s^2.
 */
bool Chassis::setBeta2DotBounds(Bounds bounds)
{
  if (bounds.size() == n_)
  {
    beta_2dot_bounds_ = bounds;
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @param phi_dot_bounds Min/max allowable value for rotation rate of module wheels, in rad/s
 */
bool Chassis::setPhiDotBounds(Bounds bounds)
{
  if (bounds.size() == n_)
  {
    phi_dot_bounds_ = bounds;
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @param phi_2dot_bounds  Min/max allowable value for the angular acceleration of the module wheels, in rad/s^2.
 */
bool Chassis::setPhi2DotBounds(Bounds bounds)
{
  if (bounds.size() == n_)
  {
    phi_2dot_bounds_ = bounds;
    return true;
  }
  else
  {
    return false;
  }
}

/**
 * @brief Calculate wheel positions from ICR.
 *
 * Computes the point in the joint space (space of all beta steering angle values) associated with a particular ICR.
 *
 * @param lambda the ICR to compute the point for.
 * @return Eigen::VectorXd row vector of beta values.
 */
VectorXd Chassis::betas(const Lambda& lambda) const
{
  auto y = a_orth_.transpose() * lambda;
  auto x = (a_ - l_).transpose() * lambda;
  // the steering angles array to be returned
  VectorXd betas(n_);

  // Uncomment the line below to return values in the
  // range [-pi, pi]
  // Otherwise this will return [-pi/2, pi]
  // betas = y.binaryExpr(x, [] (double y, double x) { return std::atan2(y,x);} ).matrix(); return betas;

  for (unsigned int idx = 0; idx < n_; ++idx)
  {
    if (abs(x[idx]) < numerical_zero_thresh_)
    {
      betas[idx] = M_PI / 2;
    }
    else
    {
      betas[idx] = atan(y[idx] / x[idx]);
    }
  }
  return betas;
}

VectorXd Chassis::displacement(const VectorXd& q_init, const VectorXd& q_final) const
{
  auto diff = (q_final - q_init).array();
  auto constrained = diff.unaryExpr([](double x) {
    double angle = atan2(sin(x), cos(x));
    double mirror = atan2(sin(x + M_PI), cos(x + M_PI));
    if (abs(angle) < abs(mirror))
    {
      return angle;
    }
    else
    {
      return mirror;
    }
  });
  return constrained;
}

/**
 * @brief
 * Identify the structural singularities that may have been produced when
 * the parameters were updated (when the ICR lies on a steering axis).
 * @param lambda the ICR estimate after the parameters were updated.
 * @returns std::optional<int> if the ICR is on a structural singularity, and the wheel
 * number which the singularity is on if there is one
 */
std::optional<int> Chassis::singularity(const Lambda& lambda) const
{
  for (unsigned int idx = 0; idx < n_; ++idx)
  {
    auto s = s_.col(idx).normalized();
    if (s.isApprox(lambda))
    {
      return idx;
    }
  }
  return {};
}

/**
 * @brief
 * Calculate the distance (norm) between q and the point that lambda maps to in the joint space.
 * Used to measure how good a 'fit' lambda is for the current steering angles.
 *
 * @param q a point in the joint space (VectorXd of beta angles)
 * @param lambda the icr to map into the joint space and calculate the distance
 * @return double the norm between q and the point lambda maps to in the joint space.
 */
double Chassis::lambda_joint_dist(const VectorXd& q, const Lambda& lambda) const
{
  VectorXd q_lambda = betas(lambda);
  VectorXd distances = displacement(q, q_lambda);
  return distances.norm();
}

/**
 * @brief Calculates the s_perp (1 and 2) value from the pappers.
 *
 * @param lambda Lambda to calculate s_perp 1 and 2 based on.
 * @return std::pair<Eigen::MatrixXd, Eigen::MatrixXd> a pair of (s1, s2)
 */
std::pair<Eigen::MatrixXd, Eigen::MatrixXd> Chassis::s_perp(const Lambda& lambda)
{
  auto beta = betas(lambda);
  RowVectorXd s = beta.array().sin().matrix().transpose();
  RowVectorXd c = beta.array().cos().matrix().transpose();
  MatrixXd s_block(3, n_);
  s_block << s, s, s;
  MatrixXd c_block(3, n_);
  c_block << c, c, c;

  MatrixXd s1_lmda = ((a_ - l_).cwiseProduct(s_block) - a_orth_.cwiseProduct(c_block));

  MatrixXd s2_lmda = ((a_ - l_).cwiseProduct(c_block) + a_orth_.cwiseProduct(s_block));

  return std::pair(s1_lmda, s2_lmda);
}

Lambda Chassis::cartesian_to_lambda(double x, double y)
{
  VectorXd plane(3);
  plane << x, y, 1;
  return plane.normalized();
}

}  // namespace swervedrive
