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

  auto starting_points = e.select_starting_points(q);
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

  Lambda lambda(0,0,1);
  d = e.compute_derivatives(lambda);
  // w should be parameterised
  EXPECT_FALSE(d.w);

  lambda << 0,0,-1;
  d = e.compute_derivatives(lambda);
  // w should be parameterised
  EXPECT_FALSE(d.w);

  lambda << 0,1,0;
  d = e.compute_derivatives(lambda);
  // v should be parameterised
  EXPECT_FALSE(d.v);

  lambda << -1,0,0;
  d = e.compute_derivatives(lambda);
  // u should be parameterised
  EXPECT_FALSE(d.u);

  // test it handles a singularity - this time the one on the wheel 1m infront of robot
  lambda << Lambda(1, 0, 1).normalized();
  d = e.compute_derivatives(lambda);
  // null axis should be u (tied dot product with w, but first in order),
  // so don't check it
  VectorXd v_vec = d.v.value();
  VectorXd w_vec = d.w.value();
  // check that the first wheel for v and w have zero derivative
  EXPECT_NEAR(v_vec(0), 0, 1e-8) << "v axis should be 0 for singularity";
  EXPECT_NEAR(w_vec(0), 0, 1e-8) << "w axis should be 0 for singularity";
}

TEST_F(EstimatorTest, UpdateParameters)
{
  Estimator e(*chassis);
  Deltas deltas;
  deltas.u = 0.5;
  deltas.v = -0.5;
  deltas.w = {};

  Lambda lambda(0,0,1);
  VectorXd q(4);
  q << 0,0,0,0;

  bool diverged;
  e.update_parameters(lambda, deltas, q, diverged);
}

TEST_F(EstimatorTest, Solve)
{
  Estimator e(*chassis);
  Derivatives derivatives;
  auto u = VectorXd(4);
  u << 1,2,3,4;
  derivatives.u = u;
  auto v = VectorXd(4);
  v << -1,-2,-3,-4;
  derivatives.v = v;
  derivatives.w = {};

  Lambda lambda(0,0,1);
  VectorXd q(4);
  q << 0,0,0,0;

  e.solve(derivatives, q, lambda);
}
