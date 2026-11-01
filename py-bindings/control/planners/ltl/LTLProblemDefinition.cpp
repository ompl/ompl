#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/control/planners/ltl/LTLProblemDefinition.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersLtl_LTLProblemDefinition(nb::module_ &m)
{
    nb::class_<oc::LTLProblemDefinition, ob::ProblemDefinition>(m, "LTLProblemDefinition")
        .def(nb::init<const oc::LTLSpaceInformationPtr &>(), nb::arg("ltlsi"))
        .def("addLowerStartState", &oc::LTLProblemDefinition::addLowerStartState, nb::arg("s"))
        .def("getLowerSolutionPath", &oc::LTLProblemDefinition::getLowerSolutionPath);
}
