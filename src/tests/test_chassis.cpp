#include <gtest/gtest.h>

#include "libswervedrive/chassis.h"

TEST(ChassisTest, InitialisesDerivedMatrices) {
  using namespace Eigen;
  using namespace swervedrive;
  VectorXd alpha = VectorXd::Zero(4);
  VectorXd l = VectorXd::Constant(4, 1);
  VectorXd b = VectorXd::Constant(4, 1);
  VectorXd r = VectorXd::Constant(4, 1);
  Bounds beta_bounds, beta_dot_bounds, beta_2dot_bounds, phi_dot_bounds, phi_2dot_bounds;

  Chassis c(alpha, l, b, r,
      beta_bounds, beta_dot_bounds, beta_2dot_bounds, phi_dot_bounds, phi_2dot_bounds);
  EXPECT_EQ(c.n, 4);

  EXPECT_EQ(c.a(0,0), 1);
  EXPECT_EQ(c.a(0,1), 0);
  EXPECT_EQ(c.a(0,2), 0);

  EXPECT_EQ(c.a_orth(0,0), 0);
  EXPECT_EQ(c.a_orth(0,1), 1);
  EXPECT_EQ(c.a_orth(0,2), 0);

  EXPECT_EQ(c.l_v(0,2), 1);
}
