#include <nanobind/nanobind.h>
#include "ompl/util/PPM.h"

namespace nb = nanobind;

void init_ppm(nb::module_& m) {
    // Expose PPM class
    nb::class_<ompl::PPM>(m, "PPM")
        .def(nb::init<>())
        .def("load_file", &ompl::PPM::loadFile)
        .def("save_file", &ompl::PPM::saveFile)
        .def("get_width", &ompl::PPM::getWidth)
        .def("get_height", &ompl::PPM::getHeight)
        .def("set_width", &ompl::PPM::setWidth)
        .def("set_height", &ompl::PPM::setHeight)
        .def("get_pixels", nb::overload_cast<>(&ompl::PPM::getPixels, nb::const_))
        .def("get_pixel", nb::overload_cast<int, int>(&ompl::PPM::getPixel, nb::const_));
}