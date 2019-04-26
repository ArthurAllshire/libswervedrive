#include <vector>
#include <cmath>

#include <gtest/gtest.h>

#include <libswervedrive/time_scaler.h>

#include "chassis_fixture.h"

using namespace Eigen;
using namespace swervedrive;

class TimeScalerTest : public ::testing::Test {
};

// function to copy contained to a defined compound type (GenType)
// which is an std::vector of length n of ContainedType
template <class GenType, class ContainedType>
GenType generate_typed_vector(ContainedType contained, int n) {
    std::vector<ContainedType> containerGen;
    for (int i=0; i<n; ++i) {
        containerGen.push_back(contained);
    }
    return (GenType) containerGen;
}

Bounds generate_uniform_bounds(double limits[2], int n) {
    return generate_typed_vector<Bounds, Vector2d>(Map<Vector2d>(limits), n);
}

// time scaler has no effect on these - define them up here
double beta_bounds[2] = {-10., 10.};
double phi_dot_bounds[2] = {-10., 10.};

void assert_scaling_bounds(
    ModuleMotion module_motion,
    double beta_dot_bounds[2],
    double beta_2dot_bounds[2],
    double phi_2dot_bounds[2]
)
{

  int n = 4;
  VectorXd alpha(n);
  alpha << 0, M_PI / 2., M_PI, 3. / 2. * M_PI;  // On sides of square robot
  VectorXd l = VectorXd::Constant(n, 1);        // Side length of robot 2m
  VectorXd b = VectorXd::Constant(n, 0);        // No offset of wheels from axis
  VectorXd r = VectorXd::Constant(n, 0.5);      // D=1m -> theta rad rotation gives theta metres displacement

  // first construct chassis
  Chassis chassis(
    alpha,
    l,
    b,
    r,
    generate_uniform_bounds(beta_bounds, n),
    generate_uniform_bounds(beta_dot_bounds, n),
    generate_uniform_bounds(beta_2dot_bounds, n),
    generate_uniform_bounds(phi_dot_bounds, n),
    generate_uniform_bounds(phi_2dot_bounds, n)
  );

  TimeScaler scaler(chassis);


  int lower_beta = module_motion.beta_dot < 0;
  int upper_beta = !lower_beta;
  int lower_phi = module_motion.phi_2dot < 0;
  int upper_phi = !lower_phi;
  bool ignore_beta = abs(module_motion.beta_dot) < 1e-2;
  bool ignore_phi = abs(module_motion.phi_2dot) < 1e-2;

  Motion motion = generate_typed_vector<Motion, ModuleMotion>(module_motion, chassis.n_);

  // then get s_dot and s_2dot bounds (s_dot and s_2dot bounds)
  std::pair<ScalingBounds, ScalingBounds> bounds = scaler.compute_scaling_bounds(motion);

  if (!ignore_beta) {
      // check we satisfy equation 36a
      EXPECT_TRUE(bounds.first.lower >=
        beta_dot_bounds[lower_beta]/module_motion.beta_dot);
      EXPECT_TRUE(bounds.first.upper <=
        beta_dot_bounds[upper_beta]/module_motion.beta_dot);

      // check that we satisfy equation 36b
      double ds_upper_sq = std::pow(bounds.first.upper, 2);
      EXPECT_TRUE(bounds.second.lower >=
        (beta_2dot_bounds[lower_beta]
        - module_motion.beta_2dot * ds_upper_sq) / module_motion.beta_dot
        );
      EXPECT_TRUE(bounds.second.upper <=
        (beta_2dot_bounds[upper_beta]
        - module_motion.beta_2dot * ds_upper_sq) / module_motion.beta_dot
        );
  }
  if (!ignore_phi) {
      EXPECT_TRUE(bounds.first.upper >=
        phi_2dot_bounds[lower_phi] / module_motion.phi_2dot
      );
      EXPECT_TRUE(bounds.first.lower <=
        phi_2dot_bounds[upper_phi] / module_motion.phi_2dot
      );
  }

  // then compute scaling parameters
  ScalingParameters scale_params = scaler.compute_scaling_parameters(bounds.first, bounds.second);

  // then use this to scale the motion
  Motion scaled_motion = scaler.scale_motion(motion, scale_params);

  // then assert that it respects the motion
  for (int i = 0; i < chassis.n_; ++i) {
      EXPECT_TRUE(beta_dot_bounds[0] <= scaled_motion[i].beta_dot) << "Violated lower beta_dot bound";
      EXPECT_TRUE(scaled_motion[i].beta_dot <= beta_dot_bounds[1]) << "Violated upper beta_dot bound";
      EXPECT_TRUE(beta_2dot_bounds[0] <= scaled_motion[i].beta_2dot) << "Violated lower beta_2dot bound";
      EXPECT_TRUE(scaled_motion[i].beta_2dot <= beta_2dot_bounds[1]) << "Violated upper beta_2dot bound";
      EXPECT_TRUE(phi_2dot_bounds[0] <= scaled_motion[i].phi_2dot) << "Violated lower phi_2dot bound";
      EXPECT_TRUE(scaled_motion[i].phi_2dot <= phi_2dot_bounds[1]) << "Violated upper phi_2dot bound";
    }
}

double beta_dot[] = {-1 , 1};
double beta_2dot[] = {-1 , 1};
double phi_2dot[] = {-1 , 1};
ModuleMotion module_motion;

TEST_F(TimeScalerTest, TestPositiveInRange)
{
    module_motion.beta_dot = 0.5;
    module_motion.beta_2dot = 0.25;
    module_motion.phi_2dot = 0.25;

    assert_scaling_bounds(module_motion, beta_dot, beta_2dot, phi_2dot);
}

TEST_F(TimeScalerTest, TestNegativeInRange)
{
    module_motion.beta_dot = -5;
    module_motion.beta_2dot = -1.5;
    module_motion.phi_2dot = -1.5;

    assert_scaling_bounds(module_motion, beta_dot, beta_2dot, phi_2dot);
}

TEST_F(TimeScalerTest, TestDBetaZero)
{
    module_motion.beta_dot = 0.01;
    module_motion.beta_2dot = -1.5;
    module_motion.phi_2dot = -1.5;

    assert_scaling_bounds(module_motion, beta_dot, beta_2dot, phi_2dot);
}

TEST_F(TimeScalerTest, TestD2BetaZero)
{
    module_motion.beta_dot = 5;
    module_motion.beta_2dot = 0.01;
    module_motion.phi_2dot = -1.5;

    assert_scaling_bounds(module_motion, beta_dot, beta_2dot, phi_2dot);
}

TEST_F(TimeScalerTest, TestD2PhiZero)
{
    module_motion.beta_dot = 5;
    module_motion.beta_2dot = -1.5 ;
    module_motion.phi_2dot = 0;

    assert_scaling_bounds(module_motion, beta_dot, beta_2dot, phi_2dot);

}

TEST_F(TimeScalerTest, TestOppositeSigns)
{
    module_motion.beta_dot = 5;
    module_motion.beta_2dot = -1.5 ;
    module_motion.phi_2dot = -5;

    assert_scaling_bounds(module_motion, beta_dot, beta_2dot, phi_2dot);

}

TEST_F(TimeScalerTest, TestAllZero)
{
    module_motion.beta_dot = 0;
    module_motion.beta_2dot = 0;
    module_motion.phi_2dot = 0;

    assert_scaling_bounds(module_motion, beta_dot, beta_2dot, phi_2dot);

}