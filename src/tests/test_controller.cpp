#include <gtest/gtest.h>

#include "libswervedrive/chassis.h"
#include "chassis_fixture.h"

using std::vector;
using namespace swervedrive;

void AssertCorrectMotion(Controller* controller, Chassis* chassis,
    double vx, double vy, double vz, vector<double> beta_expected, vector<double> phi_dot_expected,
    int steps=1000) {

    vector<ModuleState> motion(4);
    controller->updateStates(Eigen::VectorXd::Zero(4), Eigen::VectorXd::Zero(4));

    // comes from the tolerane during reconfiguring
    double tol_angle = 1.0 * M_PI/180. * chassis->n_;

    for (int k = 0; k < steps; ++k) {
        auto lastState = chassis->state_;
        motion = controller->controlStep(vx, vy, vz);
        controller->updateStates(motion);
        if (chassis->state_ != lastState && chassis->state_ == chassis->RUNNING) {
            // check that the alignment of the wheels worked
            for (int i = 0; i < chassis->n_; ++i) {
                EXPECT_NEAR(beta_expected[i], motion[i].first, tol_angle) << i;
            }
        }
    }

    for (int i = 0; i < chassis->n_; ++i) {
        EXPECT_NEAR(beta_expected[i], motion[i].first, tol_angle) << i;
        EXPECT_NEAR(phi_dot_expected[i], motion[i].second, 1e-3) << i;
    }
}

TEST_F(ControllerTest, TestVxMotion) {
    double vx = 1.0;
    double vy = 0;
    double vz = 0;
    vector<double> beta_expected{M_PI/2, 0., -M_PI/2, 0.};
    vector<double> phi_dot_expected{
        vx/chassis->r_(0), vx/chassis->r_(1),
        vx/chassis->r_(2), -vx/chassis->r_(3)};
    AssertCorrectMotion(controller, chassis, vx, vy, vz, beta_expected, phi_dot_expected);
}

TEST_F(ControllerTest, TestVyMotion) {
    double vx = 0;
    double vy = 1.0;
    double vz = 0;
    vector<double> beta_expected{0, M_PI/2, 0., -M_PI/2};
    vector<double> phi_dot_expected{
        -vy/chassis->r_(0), vy/chassis->r_(1),
        vy/chassis->r_(2), vy/chassis->r_(3)};
    AssertCorrectMotion(controller, chassis, vx, vy, vz, beta_expected, phi_dot_expected);
}

TEST_F(ControllerTest, TestVzMotion) {
    double vx = 0;
    double vy = 0;
    double vz = 1.0;
    vector<double> beta_expected{0., 0., 0., 0.};
    // -ves because positive is clockwise, at the moment
    vector<double> phi_dot_expected{
        -vz/chassis->r_(0), -vz/chassis->r_(1),
        -vz/chassis->r_(2), -vz/chassis->r_(3)};
    AssertCorrectMotion(controller, chassis, vx, vy, vz, beta_expected, phi_dot_expected);
}