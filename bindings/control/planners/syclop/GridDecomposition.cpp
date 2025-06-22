#include <nanobind/nanobind.h>
#include "ompl/control/planners/syclop/GridDecomposition.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersSyclop_GridDecomposition(nb::module_& m)
{
    // TODO [oc::GridDecomposition][IMPLEMENT]
    nb::class_<oc::GridDecomposition, oc::Decomposition>(m, "GridDecomposition")
        ;
}
