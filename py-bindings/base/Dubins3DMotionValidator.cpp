#include <nanobind/nanobind.h>

#include "ompl/base/spaces/Dubins3DMotionValidator.h"
#include "ompl/base/spaces/OwenStateSpace.h"
#include "ompl/base/spaces/VanaStateSpace.h"
#include "ompl/base/spaces/VanaOwenStateSpace.h"
#include "init.h"
#include "validation.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_Dubins3DMotionValidator(nb::module_ &m)
{
    bindSiOwningMotionValidator<ob::Dubins3DMotionValidator<ob::OwenStateSpace>>(m, "OwenMotionValidator");
    bindSiOwningMotionValidator<ob::Dubins3DMotionValidator<ob::VanaStateSpace>>(m, "VanaMotionValidator");
    bindSiOwningMotionValidator<ob::Dubins3DMotionValidator<ob::VanaOwenStateSpace>>(m, "VanaOwenMotionValidator");
}
