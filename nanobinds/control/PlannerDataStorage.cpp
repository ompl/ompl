#include <nanobind/nanobind.h>
#include "ompl/control/PlannerDataStorage.h"
#include "init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_PlannerDataStorage(nb::module_& m)
{
    // TODO [oc::PlannerDataStorage][IMPLEMENT]
    nb::class_<oc::PlannerDataStorage, ob::PlannerDataStorage>(m, "PlannerDataStorage")
        ;
}
