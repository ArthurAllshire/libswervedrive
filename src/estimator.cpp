#include "libswervedrive/estimator.h"


namespace swervedrive {
Estimator::Estimator(Chassis chassis, Epsilon init) : chassis_(chassis), epsilon_(init) {};
Estimator::Estimator(Chassis chassis) : Estimator(chassis, Epsilon(0,0,0)) {};
}
