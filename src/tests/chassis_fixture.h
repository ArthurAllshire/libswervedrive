#ifndef chassis_fixture_h
#define chassis_fixture_h

#include "libswervedrive/chassis.h"

class ChassisTest : public ::testing::Test
{

protected:
  void SetUp() override
  {
    using namespace Eigen;
    using namespace swervedrive;
    VectorXd alpha(4);
    alpha << 0, M_PI / 2., M_PI, 3. / 2. * M_PI;  // On sides of square robot
    VectorXd l = VectorXd::Constant(4, 1);     // Side length of robot 2m
    VectorXd b = VectorXd::Constant(4, 0);     // No offset of wheels from axis
    VectorXd r = VectorXd::Constant(4, 0.5);   // D=1m -> theta rad rotation gives theta metres displacement
    Bounds beta_bounds, beta_dot_bounds, beta_2dot_bounds, phi_dot_bounds, phi_2dot_bounds;

    chassis = new Chassis(alpha, l, b, r, beta_bounds, beta_dot_bounds, beta_2dot_bounds, phi_dot_bounds, phi_2dot_bounds);
  }

  void TearDown() override {
    delete chassis;
  }

  swervedrive::Chassis *chassis;
};

#endif
