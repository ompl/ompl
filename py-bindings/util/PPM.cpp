#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include "ompl/util/PPM.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::util::init_PPM(nb::module_ &m)
{
    nb::class_<ompl::PPM::Color>(m, "Color")
        .def(nb::init<>())
        .def_rw("red", &ompl::PPM::Color::red)
        .def_rw("green", &ompl::PPM::Color::green)
        .def_rw("blue", &ompl::PPM::Color::blue);

    nb::class_<ompl::PPM>(m, "PPM")
        .def(nb::init<>())
        .def("loadFile", &ompl::PPM::loadFile)
        .def("saveFile", &ompl::PPM::saveFile)
        .def("getWidth", &ompl::PPM::getWidth)
        .def("getHeight", &ompl::PPM::getHeight)
        .def("setWidth", &ompl::PPM::setWidth)
        .def("setHeight", &ompl::PPM::setHeight)
        .def(
            "getPixel",
            [](ompl::PPM &self, int row, int col) -> ompl::PPM::Color &
            {
                // setWidth()/setHeight() need not match the loaded pixels, so check against both.
                const std::size_t index = static_cast<std::size_t>(row) * self.getWidth() + col;
                if (row < 0 || col < 0 || static_cast<unsigned int>(row) >= self.getHeight() ||
                    static_cast<unsigned int>(col) >= self.getWidth() || index >= self.getPixels().size())
                    throw nb::index_error("pixel coordinates out of range");
                return self.getPixel(row, col);
            },
            nb::arg("row"), nb::arg("col"), nb::rv_policy::reference_internal);
}
