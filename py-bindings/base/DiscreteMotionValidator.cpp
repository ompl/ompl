#include <nanobind/nanobind.h>

#include "ompl/base/DiscreteMotionValidator.h"
#include "init.h"
#include "validation.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_DiscreteMotionValidator(nb::module_ &m)
{
    bindSiOwningMotionValidator<ob::DiscreteMotionValidator>(m, "DiscreteMotionValidator");
}
