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
