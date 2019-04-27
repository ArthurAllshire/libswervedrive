#ifndef kinematic_model_h_
#define kinematic_model_h_

#include "libswervedrive/chassis.h"

namespace swervedrive {
class KinematicModel {
public:
  KinematicModel (const Chassis&);
  ~KinematicModel () = default;

  std::pair<Nu, Lambda> compute_chassis_motion(Lambda lambda_desired, double mu_desired,
      Lambda lamda_estimated, double mu_estimated,
      double k_backtrack, double k_lambda, double k_mu);
  double compute_mu(Lambda, double phi_dot);
  Motion compute_actuator_motion(Lambda lambda, Lambda lambda_dot, Lambda lambda_2dot,
      double mu, double mu_dot, Eigen::VectorXd betas);
  Motion reconfigure_wheels(Eigen::VectorXd betas_desired, Eigen::VectorXd betas_estimated);
  Xi compute_odometry(Lambda, double mu, double dt);
  double estimate_mu(Lambda, Eigen::VectorXd phi_dot);
private:
  Chassis chassis_;
};

}
#endif

