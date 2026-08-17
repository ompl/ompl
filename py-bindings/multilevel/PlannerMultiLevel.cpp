#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ompl/multilevel/datastructures/PlannerMultiLevel.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace ml = ompl::multilevel;

void ompl::binding::multilevel::init_PlannerMultiLevel(nb::module_ &m)
{
    nb::class_<ml::PlannerMultiLevel, ob::Planner>(m, "PlannerMultiLevel")
        .def("getProblemDefinition", &ml::PlannerMultiLevel::getProblemDefinition, nb::arg("level"),
             nb::rv_policy::reference_internal)
        .def("getProblemDefinitionVector", &ml::PlannerMultiLevel::getProblemDefinitionVector,
             nb::rv_policy::reference_internal)
        .def("getLevels", &ml::PlannerMultiLevel::getLevels)
        .def("getDimensionsPerLevel", &ml::PlannerMultiLevel::getDimensionsPerLevel);
}
