#include <gtest/gtest.h>

#include "libswervedrive/estimator.h"
#include "chassis_fixture.h"

using namespace Eigen;
using namespace swervedrive;

class EstimatorTest : public ChassisTest
{
};

TEST_F(EstimatorTest, FindStartingPoints)
{
  Estimator e(*chassis);
  VectorXd q(4);
  q << 0, 0, 0, 0;

  auto starting_points = e.selectStartingPoints(q);
  EXPECT_EQ(starting_points.size(), 4);  // Should be four intersections
  for (auto sp : starting_points)
  {
    EXPECT_TRUE(sp.isApprox(Vector3d(0, 0, 1)));
  }
}

TEST_F(EstimatorTest, ComputeDerivatives)
{
  Estimator e(*chassis);
  Derivatives d;

  Lambda lambda(0, 0, 1);
  d = e.computeDerivatives(lambda);
  // w should be parameterised
  EXPECT_FALSE(d.w);

  lambda << 0, 0, -1;
  d = e.computeDerivatives(lambda);
  // w should be parameterised
  EXPECT_FALSE(d.w);

  lambda << 0, 1, 0;
  d = e.computeDerivatives(lambda);
  // v should be parameterised
  EXPECT_FALSE(d.v);

  lambda << -1, 0, 0;
  d = e.computeDerivatives(lambda);
  // u should be parameterised
  EXPECT_FALSE(d.u);

  // test it handles a singularity - this time the one on the wheel 1m infront of robot
  // This was failing during runs
  lambda << Lambda(1, 0, 1).normalized();
  EXPECT_NO_THROW(d = e.computeDerivatives(lambda));
  // null axis should be u (tied dot product with w, but first in order),
  // so don't check it
  EXPECT_FALSE(d.u);
  VectorXd v_vec = d.v.value();
  VectorXd w_vec = d.w.value();
  // check that the first wheel for v and w have zero derivative
  EXPECT_NEAR(v_vec(0), 0, 1e-8) << "v axis should be 0 for singularity";
  EXPECT_NEAR(w_vec(0), 0, 1e-8) << "w axis should be 0 for singularity";
  EXPECT_NEAR((*d.v)(1), (*d.w)(3), 1e-2) << "dv: " << *d.v << std::endl << "dw: " << *d.w << std::endl;
  EXPECT_NEAR((*d.w)(1), (*d.v)(3), 1e-2);
}

TEST_F(EstimatorTest, UpdateParameters)
{
  Estimator e(*chassis);
  Deltas deltas;
  deltas.u = 0.5;
  deltas.v = -0.5;
  deltas.w = {};

  Lambda lambda(0, 0, 1);
  VectorXd q(4);
  q << 0, 0, 0, 0;

  bool diverged;
  e.updateParameters(lambda, deltas, q, diverged);
}

TEST_F(EstimatorTest, Solve)
{
  Estimator e(*chassis);
  Derivatives derivatives;
  auto u = VectorXd(4);
  u << 1, 2, 3, 4;
  derivatives.u = u;
  auto v = VectorXd(4);
  v << -1, -2, -3, -4;
  derivatives.v = v;
  derivatives.w = {};

  Lambda lambda(0, 0, 1);
  VectorXd q(4);
  q << 0, 0, 0, 0;

  e.solve(derivatives, q, lambda);
}

TEST_F(EstimatorTest, Estimate)
{
  Estimator e(*chassis);

  // test spot turn
  Lambda lambda(0, 0, 1);
  auto q = VectorXd(4);

  q << 0, 0, 0, 0;
  Lambda lambda_estimate = e.estimate(q);
  EXPECT_TRUE(lambda_estimate.isApprox(lambda));

  // Reversed wheels
  q = VectorXd(4);
  q << M_PI, M_PI, -M_PI, 0;
  lambda_estimate = e.estimate(q);
  EXPECT_TRUE(lambda_estimate.isApprox(lambda));

  q = VectorXd(4);
  q << 2.55561, -1.66897, 0.969476, -1.30082;
  lambda_estimate = e.estimate(q);
  EXPECT_FALSE(std::isnan(lambda_estimate[0]));
  EXPECT_FALSE(std::isnan(lambda_estimate[1]));
  EXPECT_FALSE(std::isnan(lambda_estimate[2]));

  // This crashed in the pybind11 module running in the
  // pyfrc simulator
  VectorXd alpha(4);
  alpha << M_PI * 3 / 4, -M_PI * 3 / 4, M_PI / 4, -M_PI / 4;
  VectorXd l(4);
  l << 1.414, 1.414, 1.414, 1.414;
  VectorXd r(4);
  r << 0.05, 0.05, 0.05, 0.05;
  Chassis c(alpha, l, r);
  Estimator e2(c);
  q = VectorXd(4);
  q << 5.00998, -2.03099, 3.84416, -4.13715;

  lambda_estimate = e2.estimate(q);
  EXPECT_FALSE(std::isnan(lambda_estimate[0]));
  EXPECT_FALSE(std::isnan(lambda_estimate[1]));
  EXPECT_FALSE(std::isnan(lambda_estimate[2]));
}
