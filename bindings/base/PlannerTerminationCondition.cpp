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
             nb::arg("fn"),
             "Construct a PlannerTerminationCondition from a termination function")
        .def(nb::init<const std::function< bool()> &, double>(),
             nb::arg("fn"), nb::arg("period"),
             "Construct a PlannerTerminationCondition from a termination function and a checking period")
        .def("__call__", &ob::PlannerTerminationCondition::operator(),
             "Evaluate the termination condition by calling it.")
        .def("terminate", &ob::PlannerTerminationCondition::terminate,
             "Force the termination condition to be true.")
        .def("eval", &ob::PlannerTerminationCondition::eval,
             "Evaluate the termination condition and return its boolean value.");

    // Bind non-member termination condition functions.
    m.def("plannerNonTerminatingCondition", &ob::plannerNonTerminatingCondition,
          "Return a termination condition that never terminates.");
    
    m.def("plannerAlwaysTerminatingCondition", &ob::plannerAlwaysTerminatingCondition,
          "Return a termination condition that always terminates.");
    
    m.def("plannerOrTerminationCondition", &ob::plannerOrTerminationCondition,
          nb::arg("c1"), nb::arg("c2"),
          "Return a termination condition that is true if either c1 or c2 is true.");
    
    m.def("plannerAndTerminationCondition", &ob::plannerAndTerminationCondition,
          nb::arg("c1"), nb::arg("c2"),
          "Return a termination condition that is true only if both c1 and c2 are true.");
    
    // Overloads for timedPlannerTerminationCondition.
    m.def("timedPlannerTerminationCondition", 
          nb::overload_cast<double>(&ob::timedPlannerTerminationCondition),
          nb::arg("duration"),
          "Return a termination condition that terminates after 'duration' seconds.");
    
    m.def("timedPlannerTerminationCondition", 
          nb::overload_cast<double, double>(&ob::timedPlannerTerminationCondition),
          nb::arg("duration"), nb::arg("interval"),
          "Return a termination condition that terminates after 'duration' seconds with checks every 'interval' seconds.");
    
    m.def("exactSolnPlannerTerminationCondition", &ob::exactSolnPlannerTerminationCondition,
          nb::arg("pdef"),
          "Return a termination condition that terminates if an exact solution is found.");
}