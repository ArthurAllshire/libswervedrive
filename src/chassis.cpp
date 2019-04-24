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
 * @param b Horizontal distance from the axis of rotation of each module to its contact with the gound, in m
 * @param r Radii of the wheels, in m
 * @param beta_bounds Min/max allowable value for steering angle, in rad.
 * @param beta_dot_bounds Min/max allowable value for rotation rate of modules, in rad/s
 * @param beta_2dot_bounds Min/max allowable value for the angular acceleration of the modules, in rad/s^2.
 * @param phi_dot_bounds Min/max allowable value for rotation rate of module wheels, in rad/s
 * @param phi_2dot_bounds  Min/max allowable value for the angular acceleration of the module wheels, in rad/s^2.
 */
Chassis::Chassis(VectorXd alpha, VectorXd l, VectorXd b, VectorXd r, Bounds beta_bounds, Bounds beta_dot_bounds,
                 Bounds beta_2dot_bounds, Bounds phi_dot_bounds, Bounds phi_2dot_bounds)
  : alpha_(alpha)
  , l_vector_(l)
  , b_(b)
  , r_(r)
  , beta_dot_bounds_(beta_dot_bounds)
  , beta_2dot_bounds_(beta_2dot_bounds)
  , phi_dot_bounds_(phi_dot_bounds)
  , phi_2dot_bounds_(phi_2dot_bounds)
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
  auto y = (a_orth_.transpose() * lambda);
  auto x = ((a_ - l_).transpose() * lambda);
  // the steering angles array to be returned
  VectorXd betas(n_);
  for (int idx = 0; idx < n_; ++idx)
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
      if (abs(angle) < abs(mirror)) {
        return angle;
      } else {
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
  for (int idx = 0; idx < n_; ++idx)
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

}  // namespace swervedrive
