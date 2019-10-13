#include "libswervedrive/controller.h"

namespace swervedrive
{
Controller::Controller(const Estimator& estimator, const KinematicModel& kinematic_model, const TimeScaler& time_scaler)
  : estimator_(estimator), kinematic_model_(kinematic_model), time_scaler_(time_scaler)
{
}

}  // namespace swervedrive
