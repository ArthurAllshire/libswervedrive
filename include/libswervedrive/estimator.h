#ifndef libswervedrive_estimator_h
#define libswervedrive_estimator_h

#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "libswervedrive/chassis.h"

namespace swervedrive
{
/**
 * @brief Represents the derivatives of each steering angle with respect to
 *        2/3 of u, v and w (the other has no value).
 *
 */
struct Derivatives
{
  std::optional<Eigen::VectorXd> u, v, w;
};

struct Deltas
{
  std::optional<double> u, v, w;
};
/**
 * @brief An object for estimating the ICR and rotation about it of a swerve drive.
 *
 */
class Estimator
{
public:
  Estimator(const Chassis& chassis, Epsilon init = Eigen::VectorXd::Zero(3, 1), double eta_lambda = 1e-4,
            double eta_delta = 1e-2, double min_delta_line_search = 1e-2, double max_iter_lambda = 50,
            double singularity_tolerance = 1e-3);
  ~Estimator() = default;

  Derivatives compute_derivatives(Lambda lambda);

  Lambda estimate(Eigen::VectorXd q);

  std::vector<Lambda> select_starting_points(Eigen::VectorXd q);

  Eigen::Vector3d solve(Derivatives derivatives, Eigen::VectorXd q, Lambda lambda);

  Lambda update_parameters(Lambda lambda, Deltas deltas, Eigen::VectorXd q);

protected:
  Chassis chassis_;
  Epsilon epsilon_;

  double eta_lambda_ = 1e-4;
  double eta_delta_ = 1e-2;
  double min_delta_line_search_ = 1e-2;
  double max_iter_lambda_ = 50;
  double singularity_tolerance_ = 1e-3;
};
}  // namespace swervedrive

#endif
