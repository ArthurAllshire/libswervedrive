#include <gtest/gtest.h>

#include "libswervedrive/chassis.h"
#include "chassis_fixture.h"

class TimeScalerTest : KinematicTest {
};

using namespace Eigen;
using namespace swervedrive;

TEST_F(KinematicTest, TestComputeActuatorMotion) {
    // Lambda lambda;
    // lambda << 0, 0, 1;
    // Lambda lambda_dot;
    // lambda_dot << 0, 0, 1;
    // Lambda lambda_2dot;
    // lambda_2dot << 0, 0, -1;
    // double mu = 1.0;
    // double mu_dot = 1.0;
    // VectorXd betas;
    // betas << 0, 0, 0, 0;


    // TODO: figure out how to implement test - in original beta not included in call
    /*
    beta_prime, beta_2prime, phi_dot, phi_dot_prime = kinematic_model.compute_actuators_motion(
        lmda, lmda_dot, lmda_2dot, mu, mu_dot
    )

    """
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
    phi_dot = ([-1 0 1]-[0 0 0]).[0 0 1]*1-0*0)/0.1
        = 1/0.1
        = 10
    phi_dot_prime = ([-1 0 1]-[0 0 0]).([0 0 1]*1+[0 0 1]*1)-0*0)/0.1
        = ([-1 0 1].[0 0 2])/0.1
        = 20
    """
    assert np.isclose(beta_prime[0,0], 0.0, atol=1e-2)
    assert np.isclose(beta_2prime[0,0], 0.0, atol=1e-2)
    assert np.isclose(phi_dot[0,0], 10, atol=1e-2)
    assert np.isclose(phi_dot_prime[0,0], 20, atol=1e-2)
    */
}

TEST_F(KinematicTest, TestEstimateMu) {
    Lambda lambda = chassis->cartesian_to_lambda(0, 0);
    double expected = 1.0; // rad / s
    VectorXd phi_dot(4);
    phi_dot << 1, 1, 1, 1;
    double mu = kinematicmodel->estimate_mu(lambda, phi_dot);
    EXPECT_TRUE(abs(
        mu - expected * chassis->r_(0)
        ) < 1e-2
        ) << "Calculated mu: " << mu << " Expected mu " << expected << "chassis r" << chassis->r_(0)
        << "\nLambda:\n" << lambda;

    lambda = chassis->cartesian_to_lambda(2, 0);
    expected = 1.0; // rad / s
    phi_dot << -expected * 1.0,
               -expected*std::sqrt(5.0),
               expected*3.0,
               expected*std::sqrt(5.0);
    mu = kinematicmodel->estimate_mu(lambda, phi_dot);
    EXPECT_TRUE(abs(
        mu - expected * chassis->r_(0)
        ) < 1e-2
        ) << "Calculated mu: " << mu << " Expected mu " << expected
        << "\nLambda:\n" << lambda;
}