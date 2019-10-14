#ifndef kinematic_model_h_
#define kinematic_model_h_

#include "libswervedrive/chassis.h"

namespace swervedrive
{
class KinematicModel
{
public:
  KinematicModel(const Chassis&, double k_beta);
  ~KinematicModel() = default;

  std::pair<Nu, Lambda> computeChassisMotion(Lambda lambda_desired, double mu_desired, Lambda lambda_estimated,
                                             double mu_estimated, double k_backtrack, double k_lambda, double k_mu);
  Motion computeActuatorMotion(Lambda lambda, Lambda lambda_dot, Lambda lambda_2dot, double mu, double mu_dot);
  Motion reconfigureWheels(Eigen::VectorXd betas_desired, Eigen::VectorXd betas_estimated);
  Xi computeOdometry(const Lambda& lambda, const double& mu, const double& dt);
  double estimateMu(Lambda lambda, Eigen::VectorXd phi_dot);

  double k_beta;
  Xi xi;

private:
  Chassis chassis_;
};

}  // namespace swervedrive
#endif
