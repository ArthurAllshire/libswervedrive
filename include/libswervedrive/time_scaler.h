#ifndef time_scaler_h_
#define time_scaler_h_

#include "libswervedrive/chassis.h"

namespace swervedrive {
class TimeScaler {
public:
  TimeScaler(const Chassis&);
  ~TimeScaler() = default;

private:
  Chassis chassis_;
};

}
#endif
