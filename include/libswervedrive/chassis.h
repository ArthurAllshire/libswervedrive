#ifndef chassis_h
#define chassis_h

#include <Eigen/Dense>

#include <vector>

namespace swervedrive {

//! Velocity bounds
using Bounds = std::vector<Eigen::Vector2d>;
//! Position of the robot in terms of [x, y, theta]
using Epsilon = Eigen::Vector3d;
//! ICR of the robot in terms of the spherical parametrisation (u, v, w)
using Lambda = Eigen::Vector3d;

/**
 * @brief Represents a chassis configurtaion
 * 
 */
struct Chassis {
  /**
   * @brief Construct a new Chassis object
   * 
   * @param Alpha array containing the angle to each of the modules measured counter clockwise from the x-axis in radians
   * @param l Discance from the centre of the robot to each module's steering axis, in m
   * @param b Horizontal distance from the axis of rotation of each module to its contact with the gound, in m
   * @param r Radii of the wheels, in m
   * @param beta_bounds Min/max allowable value for steering angle, in rad.
   * @param beta_dot_bounds Min/max allowable value for rotation rate of modules, in rad/s
   * @param beta_2dot_bounds Min/max allowable value for the angular acceleration of the modules, in rad/s^2.
   * @param phi_dot_bounds Min/max allowable value for rotation rate of module wheels, in rad/s
   * @param phi_2dot_bounds  Min/max allowable value for the angular acceleration of the module wheels, in rad/s^2.
   */
  Chassis(
    Eigen::VectorXd alpha,
    Eigen::VectorXd l,
    Eigen::VectorXd b,
    Eigen::VectorXd r,
    Bounds beta_bounds,
    Bounds beta_dot_bounds,
    Bounds beta_2dot_bounds,
    Bounds phi_dot_bounds,
    Bounds phi_2dot_bounds
    ):
    alpha(alpha), l(l), b(b), r(r),
    beta_dot_bounds(beta_dot_bounds), beta_2dot_bounds(beta_2dot_bounds),
    phi_dot_bounds(phi_dot_bounds), phi_2dot_bounds(phi_2dot_bounds)
  {
    n = alpha.size();
    a = Eigen::MatrixXd(3, n);
    a << cos(alpha.array()).transpose(), sin(alpha.array()).transpose(), Eigen::VectorXd::Zero(n).transpose();
    a_orth = Eigen::MatrixXd(3, n);
    a_orth << -sin(alpha.array()).transpose(), cos(alpha.array()).transpose(), Eigen::VectorXd::Zero(n).transpose();
    l_v = Eigen::MatrixXd(3, n);
    l_v << Eigen::MatrixXd::Zero(2, n), l.transpose();
  };

  //! Number of wheels
  int n;

  //! Orthogonal projection onto the H-sphere's equator of steering x-axis basis vector (each column represents one module)
  Eigen::MatrixXd a;
  //! Orthogonal projection onto the H-sphere's equator of steering y-axis basis vector (each column represents one module)
  Eigen::MatrixXd a_orth;
  //! Distance from chassis centre to each module rotation axis, reported along the W axis (each column represents one module)
  Eigen::MatrixXd l_v;

  //! Array containing the angle to each of the modules measured counter clockwise from the x-axis in radians
  Eigen::VectorXd alpha;
  //! Discance from the centre of the robot to each module's steering axis, in m
  Eigen::VectorXd l;
  //! Horizontal distance from the axis of rotation of each module to its contact with the gound, in m
  Eigen::VectorXd b;
  //! Radii of the wheels, in m
  Eigen::VectorXd r;
  //! Min/max allowable value for steering angle, in rad.
  Bounds beta_bounds;
  //! Min/max allowable value for rotation rate of modules, in rad/s
  Bounds beta_dot_bounds;
  //! Min/max allowable value for the angular acceleration of the modules, in rad/s^2.
  Bounds beta_2dot_bounds;
  //! Min/max allowable value for rotation rate of module wheels, in rad/s
  Bounds phi_dot_bounds;
  //! Min/max allowable value for the angular acceleration of the module wheels, in rad/s^2.
  Bounds phi_2dot_bounds;
};

}

#endif
