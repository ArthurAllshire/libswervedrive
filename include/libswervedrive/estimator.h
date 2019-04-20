#ifndef libswervedrive_estimator_h
#define libswervedrive_estimator_h

#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "libswervedrive/chassis.h"

namespace swervedrive {
/**
 * @brief Represents the derivatives of each steering angle with respect to
 *        2/3 of u, v and w (the other has no value).
 * 
 */
struct Derivatives {
  std::optional<Eigen::VectorXd> u, v, w;
};

/**
 * @brief An object for estimating the ICR and rotation about it of a swerve drive.
 * 
 */
class Estimator {
public:
  /**
   * @brief Construct a new Estimator object
   * 
   * @param chassis a \sa{Chassis} object specifying the configuration of the chassis to estimate
   * @param init Starting point [x, y, theta] of the chassis in the world frame. Defaults to zero.
   * @param eta_lambda threshold when we assume the algorithm has converged to the correct
   *     icr based on how much lambda has moved between sucessive iterations
   * @param eta_delta threshold below which a starting point for iterative icr iteration is
   *     selected based on the norm between the measured wheel positions and the wheel positions obtained from that starting point
   * @param min_delta_line_search minimum size of the free parameters delta_m and delta_m
   *     to avoid infinate recursion in the line search for the next iteration of the position of lambda
   * @param max_iter_lambda maximum iterations allowed for the iterative icr estimation
   *     algorithm to converge on one point
   * @param singularity_tolerance how close a point must be to be considered to be
   *     'on a structural singularity'
   */
  Estimator(Chassis chassis,
    Epsilon init = Eigen::VectorXd::Zero(3, 1),
    double eta_lambda=1e-4,
    double eta_delta=1e-2,
    double min_delta_line_search=1e-2,
    double max_iter_lambda=50,
    double singularity_tolerance=1e-3
    );
  ~Estimator() = default;

  /**
   * @brief Compute derivatives of constraining surface at lambda.
   * 
   * @param lambda Position of ICR.
   * @return Derivatives (S_u, S_v, S_w), the vectors containing the derivatives of each steering angle 
   *     with respect to u, v, and w respectively
   */
  Derivatives compute_derivatives(Lambda lambda);

  // TODO document rest of methods as they are imelemented

  /**
   * @brief 
   * 
   * @return Lambda 
   */
  Lambda estimate_lambda();
  /**
   * @brief 
   * 
   * @param lambda 
   * @return int 
   */
  int handle_singularities(Lambda lambda);
  /**
   * @brief Calculate wheel positions from ICR. 
   * 
   * Computes the point in the joint space (space of all beta steering angle values) associated with a particular ICR.
   * 
   * @param lambda the ICR to compute the point for.
   * @return Eigen::VectorXd row vector expressing the point. 
   */
  Eigen::VectorXd lambda_to_betas(Lambda lambda);
  /**
   * @brief 
   * 
   * @param q 
   * @return std::vector<Lambda> 
   */
  std::vector<Lambda> select_starting_points(Eigen::VectorXd q);
  /**
   * @brief 
   * 
   * @param derivatives 
   * @param q 
   * @param lambda 
   * @return Eigen::Vector3d 
   */
  Eigen::Vector3d solve(Derivatives derivatives, Eigen::VectorXd q, Lambda lambda);
  /**
   * @brief 
   * 
   * @param lambda 
   * @param deltas 
   * @param q 
   * @return Lambda 
   */
  Lambda update_parameters(Lambda lambda, Eigen::Vector3d deltas, Eigen::VectorXd q);

protected:
  Chassis chassis_;
  Epsilon epsilon_;

  double eta_lambda_ = 1e-4;
  double eta_delta_ = 1e-2;
  double min_delta_line_search_ = 1e-2;
  double max_iter_lambda_ = 50;
  double singularity_tolerance_ = 1e-3;
};
}

#endif
