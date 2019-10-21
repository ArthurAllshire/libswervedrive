#include <gtest/gtest.h>

#include "libswervedrive/chassis.h"
#include "chassis_fixture.h"

using namespace Eigen;
using namespace swervedrive;

TEST_F(KinematicTest, TestComputeActuatorMotion)
{
  Lambda lambda;
  lambda << 0, 0, 1;
  Lambda lambda_dot;
  lambda_dot << 0, 0, 1;
  Lambda lambda_2dot;
  lambda_2dot << 0, 0, -1;
  double mu = 1.0;
  double mu_dot = 1.0;

  Motion motion = kinematicmodel->computeActuatorMotion(lambda, lambda_dot, lambda_2dot, mu, mu_dot);

  /*
  We do the calculations by hand to check that this is correct
  Consider the first wheel
  alpha = 0
  a = [1 0 0]T
  a_orth = [0 1 0]T
  l = [0 0 1]T
  sin(beta) = [0 1 0].[0 0 1]
      = 0
  cos(beta) = ([1 0 0]-[0 0 1]).[0 0 1]
      = [1 0 -1].[0 0 1]
      = -1

  s1_lmda = sin(b)[1 0 -1] - cos(b)[0 1 0]
      = [0 0 0] - [0 1 0]
      = [0 -1 0]
  s2_lmda = cos(b)[1 0 -1] + sin(b)[0 1 0]
      = -1*[1 0 -1] + [0 0 0]
      = [-1 0 1]
  beta_prime = -[0 -1 0].[0 0 1]/[-1 0 1].[0 0 1]
      = - 0/1
      = 0
  beta_2prime = (-2*0*[-1 0 1].[0 0 1]+[0 -1 0].[0 0 -1]) /
      [-1 0 1].[0 0 1]
      = (0+0)/1
      = 0
  phi_dot = ([-1 0 1]-[0 0 0]).[0 0 1]*1-0*0)/0.5
      = 1/0.5
      = 2
  phi_dot_prime = ([-1 0 1]-[0 0 0]).([0 0 1]*1+[0 0 1]*1)-0*0)/0.5
      = ([-1 0 1].[0 0 2])/0.5
      = 4
  */

  double tol = 1e-2;
  ModuleMotion m = motion[0];
  EXPECT_TRUE(abs(m.beta_dot - 0.0) < tol) << "Beta dot " << m.beta_dot << "Expected " << 0;
  EXPECT_TRUE(abs(m.beta_2dot - 0.0) < tol) << "Beta 2dot " << m.beta_2dot << "Expected " << 0;
  EXPECT_TRUE(abs(m.phi_dot - -2.0) < tol) << "Phi Dot " << m.phi_dot << "Expected " << 2.0;
  EXPECT_TRUE(abs(m.phi_2dot - -4.0) < tol) << "Phi Dot " << m.phi_2dot << "Expected " << 4.0;
}

TEST_F(KinematicTest, TestComputesActuatorMotionReverse)
{
  // Test that reversing lambda and mu (and thus their
  // derivatives) results in no change in actuator
  // motion
  Lambda lambda;
  lambda << 0.99978376, -0.0, 0.02079483;
  Lambda lambda_dot;
  lambda_dot << 0.02162124, 0.0, -1.03951656;
  Lambda lambda_2dot;
  lambda_2dot << 0, 0, -51.98706951;
  double mu = 126.4;
  double mu_dot = 1.0;

  Motion motion = kinematicmodel->computeActuatorMotion(lambda, lambda_dot, lambda_2dot, mu, mu_dot);

  Motion motion_neg = kinematicmodel->computeActuatorMotion(-lambda, -lambda_dot, -lambda_2dot, -mu, -mu_dot);

  double tol = 1e-8;
  for (uint i = 0; i < motion.size(); ++i)
  {
    EXPECT_TRUE(abs(motion[i].beta_dot - motion_neg[i].beta_dot) < tol)
        << "Module " << i << "Beta Dot Expected: " << motion[i].beta_dot << " Negative input is "
        << motion_neg[i].beta_dot;
  }
}

TEST_F(KinematicTest, TestEstimateMu)
{
  Lambda lambda = chassis->cartesianToLambda(0, 0);
  double theta_dot = 1.0;  // rad / s
  VectorXd phi_dot(4);
  phi_dot << -theta_dot / chassis->r_[0], -theta_dot / chassis->r_[1], -theta_dot / chassis->r_[2],
      -theta_dot / chassis->r_[3];
  // theta_dot = mu * w  Controller paper eq(2)
  double expected = theta_dot / lambda[2];
  double mu = kinematicmodel->estimateMu(lambda, phi_dot);
  EXPECT_TRUE(abs(mu - expected) < 1e-2) << "Calculated mu: " << mu << " Expected mu " << expected << "chassis r"
                                         << chassis->r_(0) << "\nLambda:\n"
                                         << lambda;

  lambda = chassis->cartesianToLambda(2, 0);
  theta_dot = 1.0;  // rad / s
  phi_dot = VectorXd(4);
  phi_dot << theta_dot * 1.0 / chassis->r_[0], -theta_dot * std::sqrt(5.0) / chassis->r_[1],
      -theta_dot * 3.0 / chassis->r_[2], -theta_dot * std::sqrt(5.0) / chassis->r_[3];
  // theta_dot = mu * w  Controller paper eq(2)
  expected = theta_dot / lambda[2];
  mu = kinematicmodel->estimateMu(lambda, phi_dot);
  EXPECT_TRUE(abs(mu - expected) < 1e-2) << "Calculated mu: " << mu << " Expected mu " << expected << "\nLambda:\n"
                                         << lambda;

  lambda = chassis->cartesianToLambda(1, 1);
  theta_dot = 1.0;  // rad / s
  phi_dot = VectorXd(4);
  phi_dot << theta_dot * 1.0 / chassis->r_[0], -theta_dot * 1.0 / chassis->r_[1],
      -theta_dot * std::sqrt(5.0) / chassis->r_[2], -theta_dot * std::sqrt(5.0) / chassis->r_[3];
  // theta_dot = mu * w  Controller paper eq(2)
  expected = theta_dot / lambda[2];
  mu = kinematicmodel->estimateMu(lambda, phi_dot);
  EXPECT_TRUE(abs(mu - expected) < 1e-2) << "Calculated mu: " << mu << " Expected mu " << expected << "\nLambda:\n"
                                         << lambda;

  lambda = Lambda(0, 1, 0);  // Straight forward +ve x
  double x_dot = 1.0;        // m / s
  phi_dot = VectorXd(4);
  phi_dot << x_dot / chassis->r_[0], x_dot / chassis->r_[1], -x_dot / chassis->r_[2], -x_dot / chassis->r_[3];
  expected = x_dot;
  mu = kinematicmodel->estimateMu(lambda, phi_dot);
  EXPECT_TRUE(abs(mu - expected) < 1e-2) << "Calculated mu: " << mu << " Expected mu " << expected << "\nLambda:\n"
                                         << lambda;
}

TEST_F(KinematicTest, TestCalculateMuLimits)
{
  Lambda lambda{ 0, 0, 1 };
  auto limits = kinematicmodel->muLimits(lambda);
  EXPECT_GT(limits.second, limits.first);
  EXPECT_NEAR(limits.first, -5., 1e-2);
  EXPECT_NEAR(limits.second, 5., 1e-2);

  lambda = { 0, 1, 0 };  // Drive forward
  limits = kinematicmodel->muLimits(lambda);
  EXPECT_GT(limits.second, limits.first);
  EXPECT_NEAR(limits.first, -5., 1e-2);
  EXPECT_NEAR(limits.second, 5., 1e-2);

  lambda = { 0, -1, 0 };  // Drive forward
  limits = kinematicmodel->muLimits(lambda);
  EXPECT_GT(limits.second, limits.first);
  EXPECT_NEAR(limits.first, -5., 1e-2);
  EXPECT_NEAR(limits.second, 5., 1e-2);
}

TEST_F(KinematicTest, TestReconfigureWheels)
{
  // basic test - TODO expand for wrapping etc once we nail down
  // ranges etc to ensure it works with wrapping
  chassis->state_ = Chassis::RECONFIGURING;

  // pi/8 chosen to avoid any wrapping issues (only pi/4 difference,
  // so under pi/2)
  VectorXd beta_d(4);
  beta_d << -M_PI / 8, -M_PI / 8, -M_PI / 8, -M_PI / 8;
  VectorXd beta_e(4);
  beta_e << M_PI / 8, M_PI / 8, M_PI / 8, M_PI / 8;

  double k_beta = 40;
  Motion motion = kinematicmodel->reconfigureWheels(beta_d, beta_e, k_beta);
  for (unsigned int i = 0; i < motion.size(); ++i)
  {
    double expected = (beta_d - beta_e)(i)*k_beta;
    EXPECT_TRUE(abs(expected - motion[i].beta_dot) < 1e-8)
        << "Module " << i << " Expected " << expected << " Returned " << motion[i].beta_dot;
  }
  EXPECT_TRUE(motion.size() == 4) << "Returned Motion object has " << motion.size() << " elements. Expected " << 4;
}
