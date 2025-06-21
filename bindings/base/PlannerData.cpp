#include <nanobind/nanobind.h>
#include "ompl/base/PlannerData.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_PlannerData(nb::module_& m)
{
    // TODO [ob::PlannerDataVertex][IMPLEMENT]
    nb::class_<ob::PlannerDataVertex>(m, "PlannerDataVertex")
        ;

    // TODO [ob::PlannerDataEdge][IMPLEMENT]
    nb::class_<ob::PlannerDataEdge>(m, "PlannerDataEdge")
        ;

    // TODO [ob::PlannerData][IMPLEMENT]
    nb::class_<ob::PlannerData>(m, "PlannerData")
        ;
}
