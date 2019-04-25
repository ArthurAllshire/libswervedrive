#ifndef controller_h_
#define controller_h_

#include "libswervedrive/estimator.h"
#include "libswervedrive/kinematic_model.h"
#include "libswervedrive/time_scaler.h"

namespace swervedrive {
class Controller {
public:
  Controller(const Estimator&, const KinematicModel&, const TimeScaler&);
  ~Controller() = default;

private:
  Estimator estimator_;
  KinematicModel kinematic_model_;
  TimeScaler time_scaler_;
};

}
#endif

