#ifndef chassis_fixture_h
#define chassis_fixture_h

#include "libswervedrive/chassis.h"
#include "libswervedrive/kinematic_model.h"
#include "libswervedrive/controller.h"

class ChassisTest : public ::testing::Test
{
protected:
  virtual void SetUp() override
  {
    using namespace Eigen;
    using namespace swervedrive;
    VectorXd alpha(4);
    alpha << 0, M_PI / 2., M_PI, 3. / 2. * M_PI;  // On sides of square robot
    VectorXd l = VectorXd::Constant(4, 1);        // Side length of robot 2m
    VectorXd b = VectorXd::Constant(4, 0);        // No offset of wheels from axis
    VectorXd r = VectorXd::Constant(4, 0.5);      // D=1m -> theta rad rotation gives theta metres displacement

    Vector2d beta_bounds_arr = { -10., 10. };
    Vector2d phi_dot_bounds_arr = { -10., 10. };
    Vector2d beta_dot_bounds_arr = { -1, 1 };
    Vector2d beta_2dot_bounds_arr = { -1, 1 };
    Vector2d phi_2dot_bounds_arr = { -1, 1 };
    Bounds beta_bounds = Bounds(4, beta_bounds_arr);
    Bounds beta_dot_bounds = Bounds(4, beta_dot_bounds_arr);
    Bounds beta_2dot_bounds = Bounds(4, beta_2dot_bounds_arr);
    Bounds phi_dot_bounds = Bounds(4, phi_dot_bounds_arr);
    Bounds phi_2dot_bounds = Bounds(4, phi_2dot_bounds_arr);

    chassis = new Chassis(alpha, l, r, b);
    chassis->setBetaBounds(beta_bounds);
    chassis->setBetaDotBounds(beta_dot_bounds);
    chassis->setBeta2DotBounds(beta_2dot_bounds);
    chassis->setPhiDotBounds(phi_dot_bounds);
    chassis->setPhi2DotBounds(phi_2dot_bounds);
  }

  virtual void TearDown() override
  {
    delete chassis;
  }

  swervedrive::Chassis* chassis;
};

class KinematicTest : public ChassisTest
{
protected:
  void SetUp() override
  {
    using namespace swervedrive;

    ChassisTest::SetUp();

    kinematicmodel = new KinematicModel(*chassis);
  }

  void TearDown() override
  {
    ChassisTest::SetUp();
    delete kinematicmodel;
  }

  swervedrive::KinematicModel* kinematicmodel;
};

class ControllerTest : public ChassisTest
{
protected:
  void SetUp() override
  {
    using namespace swervedrive;

    ChassisTest::SetUp();

    controller = new Controller(*chassis);
  }

  void TearDown() override
  {
    ChassisTest::SetUp();
    delete controller;
  }

  swervedrive::Controller* controller;
};

#endif
