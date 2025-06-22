#include <nanobind/nanobind.h>
#include "ompl/control/planners/syclop/SyclopEST.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersSyclop_SyclopEST(nb::module_& m)
{
    // TODO [oc::SyclopEST][IMPLEMENT]
    nb::class_<oc::SyclopEST, oc::Syclop>(m, "SyclopEST")
        ;
}
