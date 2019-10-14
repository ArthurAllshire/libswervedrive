#include "libswervedrive/controller.h"

#include "libswervedrive/chassis.h"

using namespace Eigen;
using namespace std;

namespace swervedrive
{
Controller::Controller(const Chassis& chassis) : chassis_(chassis)
{
  // Create sensible defaults for the private members
  // User can replace them if they need to change something
  estimator_ = make_shared<Estimator>(chassis);
  kinematic_model_ = make_shared<KinematicModel>(chassis, 1.0);
  time_scaler_ = make_shared<TimeScaler>(chassis);
}

void Controller::setEstimator(shared_ptr<Estimator> estimator)
{
  estimator_ = estimator;
}

void Controller::setKinematicModel(shared_ptr<KinematicModel> kinematic_model)
{
  kinematic_model_ = kinematic_model;
}

void Controller::setTimeScaler(shared_ptr<TimeScaler> time_scaler)
{
  time_scaler_ = time_scaler;
}

void Controller::updateStates(VectorXd beta, VectorXd phi_dot)
{
  beta_ = beta;
  phi_dot_ = phi_dot;
}

void Controller::updateStates(vector<ModuleState> states)
{
  unsigned int count = 0;
  VectorXd beta(states.size()), phi_dot(states.size());
  for (auto state : states)
  {
    beta(count) = state.first;
    phi_dot(count) = state.second;
    count++;
  }
  updateStates(beta, phi_dot);
}

vector<ModuleState> Controller::controlStep(double x_dot, double y_dot, double theta_dot)
{
  vector<ModuleState> control;
  // Convert to lambda and mu
  // Eq (1)
  Lambda lambda_desired = { -y_dot, x_dot, theta_dot };
  double mu_desired = lambda_desired.norm();
  lambda_desired.normalize();

  if (chassis_.state_ == Chassis::RECONFIGURING)
  {
    auto beta_desired = chassis_.betas(lambda_desired);
    auto motion = kinematic_model_->reconfigureWheels(beta_desired, beta_);
    control = integrateMotion(motion);
    return control;
  }

  Lambda lambda_estimated = estimator_->estimate(beta_);
  double mu_estimated = kinematic_model_->estimateMu(phi_dot_, lambda_estimated);
  chassis_.xi_ = kinematic_model_->computeOdometry(lambda_estimated, mu_estimated, dt_);

  if (chassis_.state_ == Chassis::STOPPING)
  {
    mu_desired = 0;
    lambda_desired = lambda_estimated;
  }

  double k_b = 1;
  bool backtrack = true;
  ScalingBounds s_dot, s_2dot;
  Motion motion;
  while (backtrack)
  {
    auto derivs = kinematic_model_->computeChassisMotion(lambda_desired, mu_desired, lambda_estimated, mu_estimated,
                                                         k_b, 4.0, 4.0);
    Lambda lambda_dot = derivs.first.head(3);
    double mu_dot = derivs.first(3);
    Lambda lambda_2dot = derivs.second;
    motion = kinematic_model_->computeActuatorMotion(lambda_estimated, lambda_dot, lambda_2dot, mu_estimated, mu_dot);
    auto bounds = time_scaler_->computeScalingBounds(motion);
    s_dot = bounds.first;
    s_2dot = bounds.second;
    if (s_dot.lower <= s_dot.upper && s_2dot.lower <= s_2dot.upper)
    {
      backtrack = false;
    }
    else
    {
      k_b /= 2;
    }
  }

  auto scaling_params = time_scaler_->computeScalingParameters(s_dot, s_2dot);
  auto scaled_motion = time_scaler_->scaleMotion(motion, scaling_params);
  control = integrateMotion(scaled_motion);
  return control;
}

}  // namespace swervedrive
