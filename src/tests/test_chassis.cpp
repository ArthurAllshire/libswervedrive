#include <gtest/gtest.h>

#include "libswervedrive/chassis.h"
#include "chassis_fixture.h"

TEST_F(ChassisTest, InitialisesDerivedMatrices)
{
  EXPECT_EQ(chassis->n, 4);

  EXPECT_EQ(chassis->a(0, 0), 1);
  EXPECT_EQ(chassis->a(1, 0), 0);
  EXPECT_EQ(chassis->a(2, 0), 0);

  EXPECT_EQ(chassis->a_orth(0, 0), 0);
  EXPECT_EQ(chassis->a_orth(1, 0), 1);
  EXPECT_EQ(chassis->a_orth(2, 0), 0);

  EXPECT_EQ(chassis->l_v(2, 0), 1);
}
