#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/ProblemDefinition.h"
#include "ompl/multilevel/datastructures/PlannerMultiLevel.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ml = ompl::multilevel;

void ompl::binding::multilevel::init_PlannerMultiLevel(nb::module_ &m)
{
    nb::class_<ml::PlannerMultiLevel, ob::Planner>(m, "PlannerMultiLevel")
        // Re-bound because the per-level overload hides Planner.getProblemDefinition().
        .def("getProblemDefinition", [](ml::PlannerMultiLevel &self) -> ob::ProblemDefinitionPtr
             { return static_cast<ob::Planner &>(self).getProblemDefinition(); })
        .def("getProblemDefinition", &ml::PlannerMultiLevel::getProblemDefinition, nb::arg("level"))
        .def("getProblemDefinitionVector", &ml::PlannerMultiLevel::getProblemDefinitionVector)
        .def("getLevels", &ml::PlannerMultiLevel::getLevels)
        .def("getDimensionsPerLevel", &ml::PlannerMultiLevel::getDimensionsPerLevel);
}
