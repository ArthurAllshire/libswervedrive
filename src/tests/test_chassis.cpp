#include <gtest/gtest.h>

#include "libswervedrive/chassis.h"
#include "chassis_fixture.h"

TEST_F(ChassisTest, InitialisesDerivedMatrices)
{
  EXPECT_EQ(chassis->n_, 4);

  EXPECT_EQ(chassis->a_(0, 0), 1);
  EXPECT_EQ(chassis->a_(1, 0), 0);
  EXPECT_EQ(chassis->a_(2, 0), 0);

  EXPECT_EQ(chassis->a_orth_(0, 0), 0);
  EXPECT_EQ(chassis->a_orth_(1, 0), 1);
  EXPECT_EQ(chassis->a_orth_(2, 0), 0);

  EXPECT_EQ(chassis->l_(2, 0), 1);
}

TEST_F(ChassisTest, ConvertsLambdaToBetas)
{
  using namespace Eigen;
  using namespace swervedrive;

  Lambda lambda_spot_turn = Lambda(0, 0, 1);

  auto q = chassis->betas(lambda_spot_turn);
  EXPECT_EQ(q.size(), 4);
  VectorXd expected = VectorXd::Zero(4);
  EXPECT_TRUE(q.isApprox(expected)) << "Expected: " << expected << "\nCalculated: " << q;

  // test an icr on a structural singularity
  Lambda lambda_singular = Lambda(1, 0, 1).normalized();
  q = chassis->betas(lambda_singular);
  double expected_singular_wheel = M_PI / 2.0;
  EXPECT_NEAR(expected_singular_wheel, q(0), 1e-8) << "Expected: " << expected_singular_wheel << "\nCalculated: " << q;

  Lambda drive_along_x = Lambda(0, 1, 0);
  q = chassis->betas(drive_along_x);
  expected << M_PI / 2.0, 0, M_PI / 2.0, 0;
  EXPECT_TRUE(q.isApprox(expected)) << "Expected: " << expected << "\nCalculated: " << q;
}
