#include <gtest/gtest.h>

#include "libswervedrive/chassis.h"
#include "libswervedrive/estimator.h"

TEST(EstimatorTest, ConvertsLambdaToQ) {
  using namespace Eigen;
  using namespace swervedrive;
  VectorXd alpha(4);
  alpha << 0, M_PI/2, M_PI, 3.0*M_PI/2.0;
  VectorXd l = VectorXd::Constant(4, 1);
  VectorXd b = VectorXd::Constant(4, 1);
  VectorXd r = VectorXd::Constant(4, 1);
  Bounds beta_bounds, beta_dot_bounds, beta_2dot_bounds, phi_dot_bounds, phi_2dot_bounds;

  Chassis c(alpha, l, b, r,
      beta_bounds, beta_dot_bounds, beta_2dot_bounds, phi_dot_bounds, phi_2dot_bounds);
  Estimator e(c);

  Lambda lambda_spot_turn = Lambda(0, 0, 1);

  auto q = e.lambda_to_betas(lambda_spot_turn);
  EXPECT_EQ(q.size(), 4);
  VectorXd expected = VectorXd::Zero(4);
  EXPECT_TRUE(q.isApprox(expected)) << "Expected: " << expected << "\nCalculated: " << q;

  // test an icr on a structural singularity
  Lambda lambda_singular = Lambda(1, 0, 1).normalized();
  q = e.lambda_to_betas(lambda_singular);
  double expected_singular_wheel = M_PI/2.0;
  EXPECT_NEAR(expected_singular_wheel, q(0), 1e-8)
    << "Expected: " << expected_singular_wheel << "\nCalculated: " << q;

  Lambda drive_along_x = Lambda(0, 1, 0);
  q = e.lambda_to_betas(drive_along_x);
  expected << M_PI/2.0, 0, M_PI/2.0, 0;
  EXPECT_TRUE(q.isApprox(expected)) << "Expected: " << expected << "\nCalculated: " << q;
}
