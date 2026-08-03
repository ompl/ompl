#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <sstream>

#include "ompl/geometric/PathHybridization.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::init_PathHybridization(nb::module_ &m)
{
    nb::class_<og::PathHybridization>(m, "PathHybridization")
        .def(nb::init<ob::SpaceInformationPtr>(), nb::arg("si"))
        .def(nb::init<ob::SpaceInformationPtr, ob::OptimizationObjectivePtr>(), nb::arg("si"), nb::arg("obj"))
        .def("getHybridPath", &og::PathHybridization::getHybridPath, nb::rv_policy::reference_internal)
        .def("computeHybridPath", &og::PathHybridization::computeHybridPath)
        .def("recordPath", &og::PathHybridization::recordPath, nb::arg("pp"), nb::arg("matchAcrossGaps"))
        .def("pathCount", &og::PathHybridization::pathCount)
        .def("matchPaths", &og::PathHybridization::matchPaths, nb::arg("p"), nb::arg("q"), nb::arg("gapValue"),
             nb::arg("indexP"), nb::arg("indexQ"))
        .def("clear", &og::PathHybridization::clear)
        .def("__repr__",
             [](const og::PathHybridization &self)
             {
                 std::ostringstream oss;
                 self.print(oss);
                 return oss.str();
             })
        .def("getName", &og::PathHybridization::getName, nb::rv_policy::reference_internal);
}
