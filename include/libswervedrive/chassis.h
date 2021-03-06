#ifndef chassis_h
#define chassis_h

#include <Eigen/Dense>

#include <vector>
#include <optional>
#include <utility>

namespace swervedrive
{
//! Velocity bounds
using Bounds = std::vector<Eigen::Vector2d>;
//! ICR of the robot in terms of the spherical parametrisation (u, v, w)
using Lambda = Eigen::Vector3d;
using Nu = Eigen::Vector4d;

//! Position of the robot in terms of [x, y, theta]
using Xi = Eigen::Vector3d;

struct ModuleMotion
{
  double beta_dot;
  double beta_2dot;
  double phi_dot;
  double phi_2dot;
};

using Motion = std::vector<ModuleMotion>;

/**
 * @brief Represents a chassis configurtaion
 *
 */
class Chassis
{
public:
  Chassis(Eigen::VectorXd alpha, Eigen::VectorXd l, Eigen::VectorXd b, Eigen::VectorXd r, Bounds beta_bounds,
          Bounds beta_dot_bounds, Bounds beta_2dot_bounds, Bounds phi_dot_bounds, Bounds phi_2dot_bounds);
  ~Chassis() = default;

  Eigen::VectorXd betas(const Lambda& lambda) const;
  double lambda_joint_dist(const Eigen::VectorXd& q, const Lambda& lambda) const;
  Eigen::VectorXd displacement(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) const;
  std::optional<int> singularity(const Lambda& lambda) const;
  std::pair<Eigen::MatrixXd, Eigen::MatrixXd> s_perp(const Lambda& lambda);

  Lambda cartesian_to_lambda(double x, double y);

  //! Number of wheels
  int n_;

  //! Orthogonal projection onto the H-sphere's equator of steering x-axis basis vector (each column represents one
  //! module)
  Eigen::MatrixXd a_;
  //! Orthogonal projection onto the H-sphere's equator of steering y-axis basis vector (each column represents one
  //! module)
  Eigen::MatrixXd a_orth_;
  //! Vectors through steering axes
  Eigen::MatrixXd s_;
  //! Distance from chassis centre to each module rotation axis, reported along the W axis (each column represents one
  //! module)
  Eigen::MatrixXd l_;
  //! Distance from the rotation axis of each module to its wheel's contact with the ground (each column represents one
  //! module)
  Eigen::MatrixXd b_;

  //! Array containing the angle to each of the modules measured counter clockwise from the x-axis in radians
  Eigen::VectorXd alpha_;
  //! Distance from the centre of the robot to each module's steering axis, in m
  Eigen::VectorXd l_vector_;
  //! Horizontal distance from the axis of rotation of each module to its contact with the gound, in m
  Eigen::VectorXd b_vector_;
  //! Radii of the wheels, in m
  Eigen::VectorXd r_;
  //! Min/max allowable value for steering angle, in rad.
  Bounds beta_bounds_;
  //! Min/max allowable value for rotation rate of modules, in rad/s
  Bounds beta_dot_bounds_;
  //! Min/max allowable value for the angular acceleration of the modules, in rad/s^2.
  Bounds beta_2dot_bounds_;
  //! Min/max allowable value for rotation rate of module wheels, in rad/s
  Bounds phi_dot_bounds_;
  //! Min/max allowable value for the angular acceleration of the module wheels, in rad/s^2.
  Bounds phi_2dot_bounds_;

private:
  //! Threshold to prevent division by (nearly) zero
  const double numerical_zero_thresh_ = 1e-5;
};

}  // namespace swervedrive

#endif
