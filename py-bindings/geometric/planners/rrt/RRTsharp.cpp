#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include "ompl/geometric/planners/rrt/RRTsharp.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/PlannerTerminationCondition.h"
#include "PlannerSolve.h"
#include "../../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_RRTsharp(nb::module_ &m)
{
    nb::class_<og::RRTsharp, og::RRTXstatic>(m, "RRTsharp")
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        .def("solve", ompl::binding::solveBinding<og::RRTsharp>(), nb::arg("what"))
        .def("solve", ompl::binding::solveFnBinding<og::RRTsharp>(), nb::arg("terminationConditionFn"),
             nb::arg("checkInterval") = 0.0)
        .def(
            "getPlannerData", [](const og::RRTsharp &self, ob::PlannerData &data) { self.getPlannerData(data); },
            nb::arg("data"))
        .def("clear", &og::RRTXstatic::clear)
        .def("setup", &og::RRTsharp::setup)
        .def("setEpsilon", &og::RRTsharp::setEpsilon, nb::arg("epsilon"));
}
