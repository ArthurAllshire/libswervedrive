#include "gtest/gtest.h"

#include <fenv.h>

int main(int argc, char** argv)
{
  feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);

  ::testing::InitGoogleTest(&argc, argv);

  return RUN_ALL_TESTS();
}
