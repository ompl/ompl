#include <nanobind/nanobind.h>
#include "ompl/base/PlannerDataStorage.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_PlannerDataStorage(nb::module_& m)
{
    // TODO [ob::PlannerDataStorage][IMPLEMENT]
    // TODO [ob::PlannerDataStorage][TRAMPOLINE]
    nb::class_<ob::PlannerDataStorage>(m, "PlannerDataStorage")
        ;

    
}
