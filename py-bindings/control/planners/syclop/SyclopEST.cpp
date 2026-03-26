#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/control/planners/syclop/SyclopEST.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersSyclop_SyclopEST(nb::module_ &m)
{
    nb::class_<oc::SyclopEST, oc::Syclop>(m, "SyclopEST")
        .def(nb::init<const oc::SpaceInformationPtr &, const oc::DecompositionPtr &>());
}
