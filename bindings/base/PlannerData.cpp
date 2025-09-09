#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <iostream>
#include <sstream>
#include "ompl/base/PlannerData.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_PlannerData(nb::module_ &m)
{
    // TODO [ob::PlannerDataVertex][IMPLEMENT]
    nb::class_<ob::PlannerDataVertex>(m, "PlannerDataVertex");

    // TODO [ob::PlannerDataEdge][IMPLEMENT]
    nb::class_<ob::PlannerDataEdge>(m, "PlannerDataEdge");

    // TODO [ob::PlannerData][IMPLEMENT]
    nb::class_<ob::PlannerData>(m, "PlannerData")
        .def(nb::init<const ob::SpaceInformationPtr>())
        .def("printGraphML", [](const ob::PlannerData &d) { 
            // d.printGraphML(std::cout); 
            std::ostringstream oss;
            d.printGraphML(oss);
            return oss.str();
        });
}
