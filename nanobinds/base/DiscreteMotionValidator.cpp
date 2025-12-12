#include <nanobind/nanobind.h>
#include "ompl/base/DiscreteMotionValidator.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_DiscreteMotionValidator(nb::module_& m)
{
    // TOOD [ob::DiscreteMotionValidator][IMPLEMENT]
    nb::class_<ob::DiscreteMotionValidator, ob::MotionValidator>(m, "DiscreteMotionValidator")
        ;

}
