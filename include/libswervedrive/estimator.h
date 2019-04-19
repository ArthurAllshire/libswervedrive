#ifndef libswervedrive_estimator_h
#define libswervedrive_estimator_h

#include <optional>
#include <vector>

#include <Eigen/Dense>

#include "libswervedrive/chassis.h"

namespace swervedrive {
struct Derivatives {
  std::optional<Eigen::VectorXd> u, v, w;
};


class Estimator {
public:
  Estimator(Chassis chassis,
    Epsilon init = Eigen::VectorXd::Zero(3, 1),
    double eta_lambda=1e-4,
    double eta_delta=1e-2,
    double min_delta_line_search=1e-2,
    double max_iter_lambda=50,
    double singularity_tolerance=1e-3
    );
  ~Estimator() = default;

  // compute the derivatives of the constraining surface in the space of all possible wheel
  // configurations at lambda
  // returns (S_u, S_v, S_w), the vectors containing the derivatives of each steering angle
  // with respect to u, v, and w respectively
  Derivatives compute_derivatives(Lambda lambda);
  Lambda estimate_lambda();
  int handle_singularities(Lambda lambda);
  Eigen::VectorXd lambda_to_betas(Lambda lambda);
  std::vector<Lambda> select_starting_points(Eigen::VectorXd q);
  Eigen::Vector3d solve(Derivatives derivatives, Eigen::VectorXd q, Lambda lambda);
  Lambda update_parameters(Lambda lambda, Eigen::Vector3d deltas, Eigen::VectorXd q);

protected:
  Chassis chassis_;
  Epsilon epsilon_;

  // threshold when we assume the algorithm has converged to the correct icr based on how much
  // lambda has moved between sucessive iterations
  double eta_lambda_ = 1e-4;
  // threshold below which a starting point for iterative icr iteration is selected based on the norm
  // between the measured wheel positions and the wheel positions obtained from that starting point
  double eta_delta_ = 1e-2;
  // minimum size of the free parameters delta_m and delta_m to avoid infinate recursion in the line
  // search for the next iteration of the position of lambda
  double min_delta_line_search_ = 1e-2;
  // maximum iterations allowed for the iterative icr estimation algorithm to converge on one point
  double max_iter_lambda_ = 50;
  // how close a point must be to be considered to be 'on a structural singularity'
  double singularity_tolerance_ = 1e-3;
};
}

#endif
