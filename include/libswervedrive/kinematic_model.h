#ifndef kinematic_model_h_
#define kinematic_model_h_

#include "libswervedrive/chassis.h"

namespace swervedrive
{
class KinematicModel
{
public:
  KinematicModel(Chassis&, double k_beta);
  ~KinematicModel() = default;

  std::pair<Nu, Lambda> computeChassisMotion(const Lambda& lambda_desired, const double& mu_desired,
                                             const Lambda& lambda_estimated, const double& mu_estimated,
                                             const double& k_backtrack, const double& k_lambda, const double& k_mu);
  Motion computeActuatorMotion(const Lambda& lambda, const Lambda& lambda_dot, const Lambda& lambda_2dot,
                               const double& mu, const double& mu_dot);
  Motion reconfigureWheels(const Eigen::VectorXd& betas_desired, const Eigen::VectorXd& betas_estimated);
  Xi computeOdometry(const Lambda& lambda, const double& mu, const double& dt);
  double estimateMu(const Lambda& lambda, const Eigen::VectorXd& phi_dot);

  double k_beta;
  Xi xi;

private:
  Chassis& chassis_;
};

}  // namespace swervedrive
#endif
