#ifndef time_scaler_h_
#define time_scaler_h_

#include "libswervedrive/chassis.h"

namespace swervedrive
{
struct ScalingBounds
{
  double lower;
  double upper;
  ScalingBounds(double lower = 0, double upper = 1) : lower(lower), upper(upper)
  {
  }
};

struct ScalingParameters
{
  double s_dot;
  double s_2dot;
};

class TimeScaler
{
public:
  TimeScaler(const Chassis&);
  ~TimeScaler() = default;

  Motion scaleMotion(const Motion&, const ScalingParameters&);
  ScalingParameters computeScalingParameters(const ScalingBounds& s_dot, const ScalingBounds& s2_dot);
  std::pair<ScalingBounds, ScalingBounds> computeScalingBounds(const Motion&);
  ScalingBounds sDotBounds(const ModuleMotion&, int module_number);
  ScalingBounds s2DotBounds(const ModuleMotion&, double s_dot, int module_number);

private:
  Chassis chassis_;

  const double ignore_beta_threshold_ = 1e-2;
  const double ignore_phi_threshold_ = 1e-2;
};

}  // namespace swervedrive
#endif
