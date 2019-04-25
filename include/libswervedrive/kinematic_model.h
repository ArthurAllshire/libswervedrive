#ifndef kinematic_model_h_
#define kinematic_model_h_

#include "libswervedrive/chassis.h"

namespace swervedrive {
class KinematicModel {
public:
  KinematicModel (const Chassis&);
  ~KinematicModel () = default;

private:
  Chassis chassis_;
};

}
#endif

