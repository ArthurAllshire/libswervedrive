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

struct ModuleMotion
{
  double beta_dot;
  double beta_2dot;
  double phi_2dot;
};

using Motion = std::vector<ModuleMotion>;

class TimeScaler
{
public:
  TimeScaler(const Chassis&);
  ~TimeScaler() = default;

  Motion scale_motion(const Motion&, const ScalingParameters&);
  ScalingParameters compute_scaling_parameters(const ScalingBounds& s_dot, const ScalingBounds& s2_dot);
  std::pair<ScalingBounds, ScalingBounds> compute_scaling_bounds(const Motion&);
  ScalingBounds s_dot_bounds(const ModuleMotion&, int module_number);
  ScalingBounds s_2dot_bounds(const ModuleMotion&, double s_dot, int module_number);

private:
  Chassis chassis_;

  const double ignore_beta_threshold_ = 1e-2;
  const double ignore_phi_threshold_ = 1e-2;
};

}  // namespace swervedrive
#endif
