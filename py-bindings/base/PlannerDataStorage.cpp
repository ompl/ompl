#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include "ompl/base/PlannerDataStorage.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_PlannerDataStorage(nb::module_ &m)
{
    nb::class_<ob::PlannerDataStorage>(m, "PlannerDataStorage")
        .def(nb::init<>())
        .def("store", nb::overload_cast<const ob::PlannerData &, const char *>(&ob::PlannerDataStorage::store),
             nb::arg("pd"), nb::arg("filename"), "Store the PlannerData structure to the given filename.")
        .def("load", nb::overload_cast<const char *, ob::PlannerData &>(&ob::PlannerDataStorage::load),
             nb::arg("filename"), nb::arg("pd"), "Load the PlannerData structure from the given filename.");
}
