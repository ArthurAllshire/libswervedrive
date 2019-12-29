#include <gtest/gtest.h>

#include "libswervedrive/chassis.h"
#include "chassis_fixture.h"

using std::vector;
using namespace swervedrive;

void AssertCorrectMotion(Controller* controller, double vx, double vy, double vz, vector<double> beta_expected, vector<double> phi_dot_expected,
    int steps=100) {

    vector<ModuleState> motion(4);
    controller->updateStates(Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(4));
    for (int i = 0; i < steps; ++i) {
        motion = controller->controlStep(vx, vy, vz);
        controller->updateStates(motion);
    }

    for (int i = 0; i < 4; ++i) {
        EXPECT_FLOAT_EQ(beta_expected[i], motion[i].first);
        EXPECT_FLOAT_EQ(phi_dot_expected[i], motion[i].second);
    }
}

TEST_F(ControllerTest, TestVxMotion) {
    double vx = 1.0;
    double vy = 0;
    double vz = 0;
    vector<double> beta_expected{M_PI/2, 0., M_PI/2, 0.};
    vector<double> phi_dot_expected{
        vx/chassis->r_(0), vx/chassis->r_(1),
        vx/chassis->r_(2), vx/chassis->r_(3)};
    AssertCorrectMotion(controller, vx, vy, vz, beta_expected, phi_dot_expected);
}

TEST_F(ControllerTest, TestVzMotion) {
    double vx = 0;
    double vy = 0;
    double vz = 1.0;
    vector<double> beta_expected{0., 0., 0., 0.};
    vector<double> phi_dot_expected{
        vz/chassis->r_(0), vz/chassis->r_(1),
        vz/chassis->r_(2), vz/chassis->r_(3)};
    AssertCorrectMotion(controller, vx, vy, vz, beta_expected, phi_dot_expected);
}