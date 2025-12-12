#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <limits>

#include "ompl/base/PlannerTerminationCondition.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/util/Time.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_PlannerTerminationCondition(nb::module_ &m)
{
    // Bind the PlannerTerminationCondition class.
    nb::class_<ob::PlannerTerminationCondition>(m, "PlannerTerminationCondition")
        .def(nb::init<const std::function< bool()> &>(),
             nb::arg("fn"))
        .def(nb::init<const std::function< bool()> &, double>(),
             nb::arg("fn"), nb::arg("period"))
        .def("__call__", &ob::PlannerTerminationCondition::operator())
        .def("terminate", &ob::PlannerTerminationCondition::terminate)
        .def("eval", &ob::PlannerTerminationCondition::eval);

    // Bind non-member termination condition functions.
    m.def("plannerNonTerminatingCondition", &ob::plannerNonTerminatingCondition);
    
    m.def("plannerAlwaysTerminatingCondition", &ob::plannerAlwaysTerminatingCondition);
    
    m.def("plannerOrTerminationCondition", &ob::plannerOrTerminationCondition,
          nb::arg("c1"), nb::arg("c2"));
    
    m.def("plannerAndTerminationCondition", &ob::plannerAndTerminationCondition,
          nb::arg("c1"), nb::arg("c2"));
    
    // Overloads for timedPlannerTerminationCondition.
    m.def("timedPlannerTerminationCondition", 
          nb::overload_cast<double>(&ob::timedPlannerTerminationCondition),
          nb::arg("duration"));
    
    m.def("timedPlannerTerminationCondition", 
          nb::overload_cast<double, double>(&ob::timedPlannerTerminationCondition),
          nb::arg("duration"), nb::arg("interval"));
    
    m.def("exactSolnPlannerTerminationCondition", &ob::exactSolnPlannerTerminationCondition,
          nb::arg("pdef"));
}