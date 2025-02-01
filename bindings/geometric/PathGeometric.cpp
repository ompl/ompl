#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/string.h>
#include "ompl/geometric/PathGeometric.h"
#include <sstream>

namespace nb = nanobind;

void initPathGeometric(nb::module_ &m) {
    nb::class_<ompl::geometric::PathGeometric, ompl::base::Path>(m, "PathGeometric")
        .def("print", [](const ompl::geometric::PathGeometric &self) {
            std::ostringstream ss;
            self.print(ss);
            return ss.str();
        })
        .def("__str__", [](const ompl::geometric::PathGeometric &self) {
            std::ostringstream ss;
            self.print(ss);
            return ss.str();
        });
}
