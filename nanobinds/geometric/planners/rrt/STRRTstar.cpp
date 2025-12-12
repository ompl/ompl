#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/geometric/planners/rrt/STRRTstar.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersRrt_STRRTstar(nb::module_& m)
{
    // TODO [og::STRRTstar][IMPLEMENT]
    // TAG [og::STRRTstar][Planner]
    nb::class_<og::STRRTstar, ob::Planner>(m, "STRRTstar")
        .def(nb::init<const ob::SpaceInformationPtr &>(),
             nb::arg("si"))
        .def("solve", 
             [](og::STRRTstar &self, const ob::PlannerTerminationCondition &ptc) {
                 return self.solve(ptc);
             },
             nb::arg("ptc"))
        .def("setRange", &og::STRRTstar::setRange, nb::arg("range"));
}
