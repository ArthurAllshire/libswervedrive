#include "libswervedrive/time_scaler.h"

namespace swervedrive
{
TimeScaler::TimeScaler(const Chassis& chassis) : chassis_(chassis)
{
}

Motion TimeScaler::scale_motion(const Motion& motion, const ScalingParameters& scaling_parameters)
{
  Motion scaled_motion;
  for (auto module_motion : motion)
  {
    ModuleMotion m;
    m.beta_dot = module_motion.beta_dot * scaling_parameters.s_dot;
    m.beta_2dot = module_motion.beta_2dot * std::pow(scaling_parameters.s_dot, 2) +
                  module_motion.beta_dot * scaling_parameters.s_2dot;
    m.phi_2dot = module_motion.phi_2dot * scaling_parameters.s_dot;
    scaled_motion.push_back(m);
  }

  return scaled_motion;
}

ScalingParameters TimeScaler::compute_scaling_parameters(const ScalingBounds& s_dot, const ScalingBounds& s_2dot)
{
  ScalingParameters sp;
  sp.s_dot = s_dot.upper;
  sp.s_2dot = s_2dot.upper;

  return sp;
}

std::pair<ScalingBounds, ScalingBounds> TimeScaler::compute_scaling_bounds(const Motion& motion)
{
  ScalingBounds s_dot, s_2dot;
  s_dot.lower = s_2dot.lower = 0;
  s_dot.upper = s_2dot.upper = 1;

  int idx = 0;  // module index
  for (auto m : motion)
  {
    auto bounds = s_dot_bounds(m, idx++);
    s_dot.lower = std::max(s_dot.lower, bounds.lower);
    s_dot.upper = std::min(s_dot.upper, bounds.upper);
  }
  idx = 0;
  for (auto m : motion)
  {
    auto bounds = s_2dot_bounds(m, s_dot.upper, idx++);
    s_2dot.lower = std::max(s_2dot.lower, bounds.lower);
    s_2dot.upper = std::min(s_2dot.upper, bounds.upper);
  }

  return std::make_pair(s_dot, s_2dot);
}

ScalingBounds TimeScaler::s_dot_bounds(const ModuleMotion& motion, int module)
{
  ScalingBounds sb;

  // Check if we are already in bounds
  if (chassis_.beta_dot_bounds_[module](0) < motion.beta_dot &&
      motion.beta_dot < chassis_.beta_dot_bounds_[module](1) &&
      chassis_.beta_2dot_bounds_[module](0) < motion.beta_2dot &&
      motion.beta_2dot < chassis_.beta_2dot_bounds_[module](1) &&
      chassis_.phi_2dot_bounds_[module](0) < motion.phi_2dot && motion.phi_2dot < chassis_.phi_2dot_bounds_[module](1))
  {
    return sb;
  }

  bool ignore_beta = abs(motion.beta_dot) < ignore_beta_threshold_;
  bool ignore_phi = abs(motion.phi_2dot) < ignore_phi_threshold_;

  int lower, upper;

  if (!ignore_beta)
  {
    if (motion.beta_dot >= 0)
    {
      lower = 0;
      upper = 1;
    }
    else
    {
      lower = 1;
      upper = 0;
    }
    sb.lower = std::max(sb.lower, chassis_.beta_dot_bounds_[module](lower) / motion.beta_dot);
    sb.upper = std::min(sb.upper, chassis_.beta_dot_bounds_[module](upper) / motion.beta_dot);
  }
  if (!ignore_phi)
  {
    if (motion.phi_2dot >= 0)
    {
      lower = 0;
      upper = 1;
    }
    else
    {
      lower = 1;
      upper = 0;
    }
    sb.lower = std::max(sb.lower, chassis_.phi_2dot_bounds_[module](lower) / motion.phi_2dot);
    sb.upper = std::min(sb.upper, chassis_.phi_2dot_bounds_[module](upper) / motion.phi_2dot);
  }

  return sb;
}

ScalingBounds TimeScaler::s_2dot_bounds(const ModuleMotion& motion, double s_dot, int module)
{
  ScalingBounds sb;
  // Check if we are already in bounds
  if (chassis_.beta_dot_bounds_[module](0) < motion.beta_dot &&
      motion.beta_dot < chassis_.beta_dot_bounds_[module](1) &&
      chassis_.beta_2dot_bounds_[module](0) < motion.beta_2dot &&
      motion.beta_2dot < chassis_.beta_2dot_bounds_[module](1))
  {
    return sb;
  }
  bool ignore_beta = abs(motion.beta_dot) < ignore_beta_threshold_;
  if (ignore_beta)
  {
    return sb;
  }

  int lower, upper;
  if (motion.beta_dot >= 0)
  {
    lower = 0;
    upper = 1;
  }
  else
  {
    lower = 1;
    upper = 0;
  }
  sb.lower = std::max(sb.lower, (chassis_.beta_2dot_bounds_[module](lower) - motion.beta_2dot * std::pow(s_dot, 2)) /
                                    motion.beta_dot);
  sb.upper = std::min(sb.upper, (chassis_.beta_2dot_bounds_[module](upper) - motion.beta_2dot * std::pow(s_dot, 2)) /
                                    motion.beta_dot);

  return sb;
}

}  // namespace swervedrive
