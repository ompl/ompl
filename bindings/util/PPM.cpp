#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ompl/util/PPM.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::util::init_PPM(nb::module_& m)
{
    nb::class_<ompl::PPM::Color>(m, "Color")
        .def(nb::init<>())
        .def_rw("red",   &ompl::PPM::Color::red)
        .def_rw("green", &ompl::PPM::Color::green)
        .def_rw("blue",  &ompl::PPM::Color::blue);

    nb::class_<ompl::PPM>(m, "PPM")
        .def(nb::init<>())
        .def("loadFile", &ompl::PPM::loadFile)
        .def("saveFile", &ompl::PPM::saveFile)
        .def("getWidth", &ompl::PPM::getWidth)
        .def("getHeight", &ompl::PPM::getHeight)
        .def("setWidth", &ompl::PPM::setWidth)
        .def("setHeight", &ompl::PPM::setHeight)
        .def("getPixel",
             [](const ompl::PPM &self, int row, int col) -> const ompl::PPM::Color& {
                 return self.getPixel(row, col);
             },
             nb::arg("row"), nb::arg("col"),
             nb::rv_policy::reference_internal);
}
