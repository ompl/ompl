#include <nanobind/nanobind.h>
#include "ompl/base/StateValidityChecker.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_StateValidityChecker(nb::module_& m)
{

    // TODO [ob::StateValidityChecker][IMPLEMENT]
    nb::class_<ob::StateValidityChecker>(m, "StateValidityChecker")
        ;

    // TODO [ob::ValidStateSampler][IMPLEMENT]
    nb::class_<ob::AllValidStateValidityChecker, ob::StateValidityChecker>(m, "AllValidStateValidityChecker")
        ;
}
