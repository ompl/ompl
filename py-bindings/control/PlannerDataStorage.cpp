#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>

#include "ompl/control/PlannerDataStorage.h"
#include "init.h"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_PlannerDataStorage(nb::module_ &m)
{
    nb::class_<oc::PlannerDataStorage, ob::PlannerDataStorage>(m, "PlannerDataStorage")
        .def(nb::init<>())
        .def("store", nb::overload_cast<const ob::PlannerData &, const char *>(&oc::PlannerDataStorage::store),
             nb::arg("pd"), nb::arg("filename"))
        .def("load", nb::overload_cast<const char *, ob::PlannerData &>(&oc::PlannerDataStorage::load),
             nb::arg("filename"), nb::arg("pd"));
}
