#include <gtest/gtest.h>

#include "libswervedrive/chassis.h"
#include "libswervedrive/estimator.h"

TEST(EstimatorTest, ConvertsLambdaToQ) {
  using namespace Eigen;
  using namespace swervedrive;
  VectorXd alpha = VectorXd::Zero(4);
  VectorXd l = VectorXd::Constant(4, 1);
  VectorXd b = VectorXd::Constant(4, 1);
  VectorXd r = VectorXd::Constant(4, 1);
  Bounds beta_bounds, beta_dot_bounds, beta_2dot_bounds, phi_dot_bounds, phi_2dot_bounds;

  Chassis c(alpha, l, b, r,
      beta_bounds, beta_dot_bounds, beta_2dot_bounds, phi_dot_bounds, phi_2dot_bounds);
  Estimator e(c);

  auto q = e.lambda_to_betas(Lambda(0, 0, 1));
  EXPECT_EQ(q.size(), 4);
  VectorXd expected = VectorXd::Zero(4);
  EXPECT_TRUE(q.isApprox(expected)) << "Expected: " << expected << " Calculated: " << q;
}
