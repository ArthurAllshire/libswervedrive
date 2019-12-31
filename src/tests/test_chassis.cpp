#include <gtest/gtest.h>

#include "libswervedrive/chassis.h"
#include "chassis_fixture.h"

using namespace Eigen;
using namespace swervedrive;

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
  Lambda lambda_spot_turn = Lambda(0, 0, 1);

  auto q = chassis->betas(lambda_spot_turn);
  EXPECT_EQ(q.size(), 4);
  VectorXd expected = VectorXd::Constant(4, 0);
  EXPECT_TRUE(q.isApprox(expected)) << "Expected: " << expected << "\nCalculated: " << q;

  // test an icr on a structural singularity
  Lambda lambda_singular = Lambda(1, 0, 1).normalized();
  q = chassis->betas(lambda_singular);
  double expected_singular_wheel = 0;
  EXPECT_NEAR(expected_singular_wheel, q(0), 1e-8) << "Expected: " << expected_singular_wheel << "\nCalculated: " << q;

  Lambda drive_along_x = Lambda(0, 1, 0);
  q = chassis->betas(drive_along_x);
  expected << M_PI / 2.0, 0, -M_PI / 2.0, 0;
  EXPECT_TRUE(q.isApprox(expected)) << "Expected: " << expected << "\nCalculated: " << q;
}

TEST_F(ChassisTest, IdentifiesSingularity)
{
  Lambda lambda = Vector3d(0, 0, 1).normalized();  // (x, y, 0)
  EXPECT_FALSE(chassis->singularity(lambda));

  lambda = Vector3d(1, 0, 1).normalized();  // (x, y, 0)
  EXPECT_TRUE(chassis->singularity(lambda));
  EXPECT_EQ(chassis->singularity(lambda), 0);

  lambda = Vector3d(0, -1, 1).normalized();  // (x, y, 0)
  EXPECT_TRUE(chassis->singularity(lambda));
  EXPECT_EQ(chassis->singularity(lambda), 3);
}

TEST_F(ChassisTest, CalculatesDisplacement)
{
  VectorXd q1(4), q2(4), expected(4);
  q1 << 0, -M_PI, 0, M_PI;
  q2 << M_PI, M_PI, -M_PI, -M_PI;
  expected << 0, 0, 0, 0;
  auto disp = chassis->displacement(q1, q2);
  EXPECT_TRUE((expected - disp).isMuchSmallerThan(1. - 4)) << expected << "\n" << disp;

  q1 << 0, -M_PI, 0, M_PI;
  q2 << M_PI * 3. / 2., M_PI * 3. / 2., -M_PI * 3. / 2., -M_PI * 3. / 2.;
  expected << M_PI / 2., M_PI / 2., -M_PI / 2., -M_PI / 2.;
  disp = chassis->displacement(q1, q2);
  EXPECT_TRUE(expected.isApprox(disp)) << expected << "\n" << disp;
}

TEST_F(ChassisTest, CalculatesLambdaJointDist)
{
  Lambda lambda(0, 0, 1);  // spot turn
  VectorXd q_lambda = VectorXd::Zero(4);
  VectorXd q_deltas(4);
  q_deltas << 0.1, -0.1, -0.1, 0.1;

  double dist_calc = chassis->lambdaJointDist(q_lambda + q_deltas, lambda);
  EXPECT_NEAR(dist_calc, q_deltas.norm(), 1e-8);

  lambda = Lambda(0, 1, 0);  // straight forward
  q_lambda << M_PI / 2, 0, M_PI / 2, 0;
  dist_calc = chassis->lambdaJointDist(q_lambda, lambda);
  EXPECT_NEAR(dist_calc, 0, 1e-8);

  // now test with error
  dist_calc = chassis->lambdaJointDist(q_lambda + q_deltas, lambda);
  EXPECT_NEAR(dist_calc, q_deltas.norm(), 1e-8);
}

TEST_F(ChassisTest, CalculatesSPerp)
{
  Lambda lambda;
  lambda << 0, 0, 1;
  auto beta = chassis->betas(lambda);

  std::pair<MatrixXd, MatrixXd> s_perp = chassis->sPerp(lambda, beta);

  MatrixXd expected_s1(3, chassis->n_);
  MatrixXd expected_s2(3, chassis->n_);
  expected_s1 << 0, 1, 0, -1, -1, 0, 1, 0, 0, 0, 0, 0;
  expected_s2 << 1, 0, -1, 0, 0, 1, 0, -1, -1, -1, -1, -1;
  EXPECT_TRUE(s_perp.first.isApprox(expected_s1, 1e-3)) << s_perp.first;
  EXPECT_TRUE(s_perp.second.isApprox(expected_s2, 1e-3)) << s_perp.second;

  // Test when ICR isn't in centre of robot!
  lambda = chassis->cartesianToLambda(1, 1);
  beta = chassis->betas(lambda);
  s_perp = chassis->sPerp(lambda, beta);

  expected_s1 = MatrixXd(3, chassis->n_);
  expected_s2 = MatrixXd(3, chassis->n_);
  expected_s1 << 1, 0, -0.447, -0.894, 0, 1, 0.894, 0.447, -1, -1, -0.447, 0.447;
  expected_s2 << 0, -1, -0.894, -0.447, 1, 0, -0.447, -0.894, 0, 0, -0.894, -0.894;
  EXPECT_TRUE(s_perp.first.isApprox(expected_s1, 1e-3)) << s_perp.first;
  EXPECT_TRUE(s_perp.second.isApprox(expected_s2, 1e-3)) << s_perp.second;
}

TEST_F(ChassisTest, ConvertsCartesianToLambda)
{
  auto lambda = chassis->cartesianToLambda(0, 0);
  EXPECT_NEAR(lambda[0], 0.0, 1e-3);
  EXPECT_NEAR(lambda[1], 0.0, 1e-3);
  EXPECT_NEAR(lambda[2], 1.0, 1e-3);
}

