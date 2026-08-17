#include <nanobind/nanobind.h>

#include "ompl/base/terminationconditions/IterationTerminationCondition.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initTerminationconditions_IterationTerminationCondition(nb::module_ &m)
{
    nb::class_<ob::IterationTerminationCondition>(m, "IterationTerminationCondition")
        .def(nb::init<unsigned int>(), nb::arg("numIterations"))
        .def("eval", &ob::IterationTerminationCondition::eval)
        .def("reset", &ob::IterationTerminationCondition::reset)
        .def("getTimesCalled", &ob::IterationTerminationCondition::getTimesCalled)
        .def("asPlannerTerminationCondition",
             [](ob::IterationTerminationCondition &itc) { return static_cast<ob::PlannerTerminationCondition>(itc); });
}
