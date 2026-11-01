#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/control/planners/ltl/LTLPlanner.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::initPlannersLtl_LTLPlanner(nb::module_ &m)
{
    nb::class_<oc::LTLPlanner, ob::Planner>(m, "LTLPlanner")
        .def(nb::init<const oc::LTLSpaceInformationPtr &, oc::ProductGraphPtr, double>(), nb::arg("si"), nb::arg("a"),
             nb::arg("exploreTime") = 0.5)
        .def("solve", ompl::binding::solveBinding<oc::LTLPlanner>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<oc::LTLPlanner>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def("clear", &oc::LTLPlanner::clear)
        .def("setup", &oc::LTLPlanner::setup)
        .def("getTree",
             [](oc::LTLPlanner &planner)
             {
                 std::vector<ob::State *> tree;
                 planner.getTree(tree);
                 return tree;
             });
}
