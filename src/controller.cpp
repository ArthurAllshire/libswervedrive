#include "libswervedrive/controller.h"

#include "libswervedrive/chassis.h"

using namespace Eigen;
using namespace std;

namespace swervedrive
{
Controller::Controller(Chassis& chassis) : chassis_(chassis)
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

void Controller::updateStates(const VectorXd& beta, const VectorXd& phi_dot)
{
  beta_ = beta;
  phi_dot_ = phi_dot;
}

void Controller::updateStates(const vector<ModuleState>& states)
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
  Lambda lambda_estimated = estimator_->estimate(beta_);
  // Convert to lambda and mu
  // Eq (1)
  Lambda lambda_desired = { -y_dot, x_dot, theta_dot };
  double mu_desired = lambda_desired.norm();
  lambda_desired.normalize();
  if (abs(mu_desired) < 1e-3)
  {
    // No motion commanded - set desired lambda to current estimate
    lambda_desired = lambda_estimated;
  }

  if (chassis_.state_ == Chassis::RECONFIGURING)
  {
    auto beta_desired = chassis_.betas(lambda_desired);
    auto motion = kinematic_model_->reconfigureWheels(beta_desired, beta_);
    control = integrateMotion(motion);
    return control;
  }

  double mu_estimated = kinematic_model_->estimateMu(lambda_estimated, phi_dot_);
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

vector<ModuleState> Controller::integrateMotion(const Motion& motion)
{
  vector<ModuleState> control;
  auto b_on_r = chassis_.b_vector_.cwiseQuotient(chassis_.r_);
  VectorXd beta_dot(chassis_.n_), beta_2dot(chassis_.n_), phi_dot(chassis_.n_), phi_2dot(chassis_.n_);
  unsigned int idx = 0;
  for (auto m : motion)
  {
    beta_dot(idx) = m.beta_dot;
    beta_2dot(idx) = m.beta_2dot;
    phi_dot(idx) = m.phi_dot;
    phi_2dot(idx) = m.phi_2dot;
    idx++;
  }
  VectorXd phi_dot_c =
      (phi_dot - b_on_r.cwiseProduct(beta_dot)) + (phi_2dot - b_on_r.cwiseProduct(beta_2dot)) * dt_;  // 40b
  // Check limits
  double fd = 1.0;
  for (auto i = 0; i < phi_dot_c.size(); i++)
  {
    double pdc = phi_dot_c(i);
    auto bounds = chassis_.phi_dot_bounds_[i];
    if (pdc * fd > bounds(1))
    {
      fd = bounds(1) / pdc;
    }
    if (pdc * fd < bounds(0))
    {
      fd = bounds(0) / pdc;
    }
  }
  // Check rotation rate limits
  VectorXd delta_beta_c = beta_dot * dt_ + 0.5 * beta_2dot * pow(dt_, 2);
  for (auto i = 0; i < delta_beta_c.size(); i++)
  {
    double dbc = delta_beta_c(i);
    auto bounds = chassis_.beta_dot_bounds_[i];
    if (dbc * fd > bounds(1) * dt_)
    {
      fd = bounds(1) * dt_ / dbc;
    }
    if (dbc * fd < bounds(0) * dt_)
    {
      fd = bounds(0) * dt_ / dbc;
    }
  }
  VectorXd beta_c = (beta_ + fd * delta_beta_c);  // 40a
  phi_dot_c *= fd;                                // 42

  // Check for wheel reconfig needed due to beta bounds
  bool stopping = false;
  for (auto i = 0; i < beta_c.size(); i++)
  {
    auto bounds = chassis_.beta_bounds_[i];
    auto bc = beta_c(i);
    if (bc < bounds(0) || bc > bounds(1))
    {
      stopping = true;
    }
  }
  if (stopping)
  {
    beta_c = beta_;  // 43
    phi_dot_c = phi_dot;
    chassis_.state_ = Chassis::STOPPING;
  }

  for (auto i = 0; i < beta_c.size(); i++)
  {
    double bc = beta_c(i);
    double pdc = phi_dot_c(i);
    control.push_back(make_pair(bc, pdc));
  }
  return control;
}

}  // namespace swervedrive
