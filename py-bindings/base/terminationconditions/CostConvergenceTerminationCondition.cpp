#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/terminationconditions/CostConvergenceTerminationCondition.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/ProblemDefinition.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initTerminationconditions_CostConvergenceTerminationCondition(nb::module_ &m)
{
    nb::class_<ob::CostConvergenceTerminationCondition, ob::PlannerTerminationCondition>(
        m, "CostConvergenceTerminationCondition")
        .def(
            "__init__",
            [](ob::CostConvergenceTerminationCondition *self, ob::ProblemDefinitionPtr pdef,
               std::size_t solutionsWindow, double epsilon)
            { new (self) ob::CostConvergenceTerminationCondition(pdef, solutionsWindow, epsilon); },
            nb::arg("pdef"), nb::arg("solutionsWindow") = 10, nb::arg("epsilon") = 0.1)
        .def("processNewSolution", &ob::CostConvergenceTerminationCondition::processNewSolution,
             nb::arg("solutionCost"));
}
