#include <gtest/gtest.h>

#include "libswervedrive/estimator.h"
#include "chassis_fixture.h"

class EstimatorTest : public ChassisTest
{
};

TEST_F(EstimatorTest, FooTest)
{
  using namespace Eigen;
  using namespace swervedrive;

  Estimator e(*chassis);
}
