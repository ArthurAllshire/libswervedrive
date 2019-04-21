#include "libswervedrive/estimator.h"

namespace swervedrive
{
Estimator::Estimator(const Chassis& chassis, Epsilon init, double eta_lambda, double eta_delta,
                     double min_delta_line_search, double max_iter_lambda, double singularity_tolerance)
  : chassis_(chassis)
  , epsilon_(init)
  , eta_lambda_(eta_lambda)
  , eta_delta_(eta_delta)
  , min_delta_line_search_(min_delta_line_search)
  , max_iter_lambda_(max_iter_lambda)
  , singularity_tolerance_(singularity_tolerance){};
}  // namespace swervedrive
