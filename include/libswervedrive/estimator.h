#ifndef libswervedrive_estimator_h
#define libswervedrive_estimator_h

#include <vector>

#include <Eigen/Dense>

#include "libswervedrive/types.h"

namespace libswervedrive {

using Epsilon = Eigen::Vector3d;
using Lambda = Eigen::Vector4d;

class Estimator {
public:
  Estimator(Eigen::VectorXd alpha, Eigen::VectorXd l);
  Estimator(Eigen::VectorXd alpha, Eigen::VectorXd l, Epsilon init);
  ~Estimator();

  Eigen::Vector3d compute_derivatives(Lambda lambda);
  Lambda estimate_lambda();
  int handle_singularities(Lambda lambda);
  Eigen::VectorXd S(Lambda lambda);
  std::vector<Lambda> select_starting_points(Eigen::VectorXd q);
  Eigen::Vector3d solve(Eigen::Vector3d derivatives, Eigen::VectorXd q, Lambda lambda);
  Lambda update_parameters(Lambda lambda, Eigen::Vector3d deltas, Eigen::VectorXd q);

protected:

};
}

#endif
