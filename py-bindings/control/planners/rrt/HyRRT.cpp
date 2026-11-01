#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/control/planners/rrt/HyRRT.h"
#include "../HybridPlanner.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace hybrid = ompl::binding::control::hybrid;

void ompl::binding::control::initPlannersRrt_HyRRT(nb::module_ &m)
{
    auto cls = hybrid::bindPlanner<oc::HyRRT>(m, "HyRRT");
    hybrid::bindMotion<oc::HyRRT::Motion>(cls).def_prop_ro(
        "root", [](const oc::HyRRT::Motion &motion) { return motion.root; }, nb::rv_policy::reference);
}
