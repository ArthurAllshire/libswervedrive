#ifndef controller_h_
#define controller_h_

#include "libswervedrive/chassis.h"
#include "libswervedrive/estimator.h"
#include "libswervedrive/kinematic_model.h"
#include "libswervedrive/time_scaler.h"

#include <memory>

namespace swervedrive
{
using ModuleState = std::pair<double, double>;

class Controller
{
public:
  Controller(const Chassis&);
  ~Controller() = default;

  void setEstimator(std::shared_ptr<Estimator>);
  void setKinematicModel(std::shared_ptr<KinematicModel>);
  void setTimeScaler(std::shared_ptr<TimeScaler>);

  void updateStates(Eigen::VectorXd beta, Eigen::VectorXd phi_dot);
  void updateStates(std::vector<ModuleState> states);
  std::vector<ModuleState> controlStep(double x_dot, double y_dot, double theta_dot);
  std::vector<ModuleState> integrateMotion(Motion motion);

  double dt_ = 0.02;

private:
  Chassis chassis_;
  std::shared_ptr<Estimator> estimator_;
  std::shared_ptr<KinematicModel> kinematic_model_;
  std::shared_ptr<TimeScaler> time_scaler_;

  Eigen::VectorXd beta_;
  Eigen::VectorXd phi_dot_;
};

}  // namespace swervedrive
#endif
