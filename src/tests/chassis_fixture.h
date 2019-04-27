#ifndef chassis_fixture_h
#define chassis_fixture_h

#include "libswervedrive/chassis.h"
#include "libswervedrive/kinematic_model.h"

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

    double beta_bounds_arr[2] = {-10., 10.};
    double phi_dot_bounds_arr[2] = {-10., 10.};
    double beta_dot_bounds_arr[2] = {-1 , 1};
    double beta_2dot_bounds_arr[2] = {-1 , 1};
    double phi_2dot_bounds_arr[2] = {-1 , 1};
    Bounds beta_bounds, beta_dot_bounds, beta_2dot_bounds, phi_dot_bounds, phi_2dot_bounds;
    for (int i = 0; i < 4; ++i) {
      beta_bounds.push_back(Map<Vector2d>(beta_bounds_arr));
      beta_dot_bounds.push_back(Map<Vector2d>(beta_dot_bounds_arr));
      beta_2dot_bounds.push_back(Map<Vector2d>(beta_2dot_bounds_arr));
      phi_dot_bounds.push_back(Map<Vector2d>(phi_dot_bounds_arr));
      phi_2dot_bounds.push_back(Map<Vector2d>(phi_2dot_bounds_arr));
    }


    chassis =
        new Chassis(alpha, l, b, r,
          beta_bounds, beta_dot_bounds, beta_2dot_bounds, phi_dot_bounds, phi_2dot_bounds
        );
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

    kinematicmodel = new KinematicModel(*chassis, 1.);

  }

  void TearDown() override
  {
    ChassisTest::SetUp();
    delete kinematicmodel;
  }

  swervedrive::KinematicModel* kinematicmodel;
  
};

#endif
