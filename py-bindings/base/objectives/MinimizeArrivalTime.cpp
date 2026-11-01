#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/objectives/MinimizeArrivalTime.h"
#include "ompl/base/OptimizationObjective.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initObjectives_MinimizeArrivalTime(nb::module_ &m)
{
    nb::class_<ob::MinimizeArrivalTime, ob::OptimizationObjective>(m, "MinimizeArrivalTime")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"));
}
