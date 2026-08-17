#include <nanobind/nanobind.h>
#include <nanobind/stl/chrono.h>
#include <nanobind/stl/string.h>

#include "ompl/util/Time.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::util::init_Time(nb::module_ &m)
{
    nb::class_<ompl::time::ProgressDisplay>(m, "ProgressDisplay")
        .def(nb::init<>())
        .def(nb::init<std::ostream &>(), nb::arg("os"))
        .def("__call__", [](ompl::time::ProgressDisplay &pd) { return ++pd; })
        .def("count", &ompl::time::ProgressDisplay::count);

    m.def("time_now", &ompl::time::now);
    m.def("time_seconds", nb::overload_cast<double>(&ompl::time::seconds), nb::arg("sec"));
    m.def("time_seconds", nb::overload_cast<const ompl::time::duration &>(&ompl::time::seconds), nb::arg("duration"));
    m.def("time_as_string", &ompl::time::as_string, nb::arg("point"));
}
