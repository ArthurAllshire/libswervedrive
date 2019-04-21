#include <gtest/gtest.h>

#include "libswervedrive/estimator.h"
#include "chassis_fixture.h"

class EstimatorTest : public ChassisTest
{
};

TEST_F(EstimatorTest, ConvertsLambdaToQ)
{
  using namespace Eigen;
  using namespace swervedrive;

  Estimator e(*chassis);

  Lambda lambda_spot_turn = Lambda(0, 0, 1);

  auto q = e.lambda_to_betas(lambda_spot_turn);
  EXPECT_EQ(q.size(), 4);
  VectorXd expected = VectorXd::Zero(4);
  EXPECT_TRUE(q.isApprox(expected)) << "Expected: " << expected << "\nCalculated: " << q;

  // test an icr on a structural singularity
  Lambda lambda_singular = Lambda(1, 0, 1).normalized();
  q = e.lambda_to_betas(lambda_singular);
  double expected_singular_wheel = M_PI / 2.0;
  EXPECT_NEAR(expected_singular_wheel, q(0), 1e-8) << "Expected: " << expected_singular_wheel << "\nCalculated: " << q;

  Lambda drive_along_x = Lambda(0, 1, 0);
  q = e.lambda_to_betas(drive_along_x);
  expected << M_PI / 2.0, 0, M_PI / 2.0, 0;
  EXPECT_TRUE(q.isApprox(expected)) << "Expected: " << expected << "\nCalculated: " << q;
}
