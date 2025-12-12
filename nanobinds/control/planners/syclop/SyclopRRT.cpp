#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/control/planners/syclop/SyclopRRT.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersSyclop_SyclopRRT(nb::module_& m)
{
    // TAG [oc::SyclopRRT][Planner]
    nb::class_<oc::SyclopRRT, oc::Syclop>(m, "SyclopRRT")
        .def(nb::init<const oc::SpaceInformationPtr &, const oc::DecompositionPtr &>());
}
