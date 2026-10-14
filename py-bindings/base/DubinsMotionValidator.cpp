#include <nanobind/nanobind.h>

#include "ompl/base/spaces/DubinsMotionValidator.h"
#include "ompl/base/spaces/DubinsStateSpace.h"
#include "ompl/base/spaces/ReedsSheppStateSpace.h"
#include "ompl/base/spaces/TrochoidStateSpace.h"
#include "init.h"
#include "validation.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_DubinsMotionValidator(nb::module_ &m)
{
    bindSiOwningMotionValidator<ob::DubinsMotionValidator<ob::DubinsStateSpace>>(m, "DubinsMotionValidator");
    bindSiOwningMotionValidator<ob::DubinsMotionValidator<ob::ReedsSheppStateSpace>>(m, "ReedsSheppMotionValidator");
    bindSiOwningMotionValidator<ob::DubinsMotionValidator<ob::TrochoidStateSpace>>(m, "TrochoidMotionValidator");
}
