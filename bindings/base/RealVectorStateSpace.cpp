#include <nanobind/nanobind.h>
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/base/StateSpace.h"

namespace nb = nanobind;

void initRealVectorStateSpace(nb::module_& m) {
    // Bind StateType
    nb::class_<ompl::base::RealVectorStateSpace::StateType>(m, "RealVectorStateType")
        .def("__getitem__", [](const ompl::base::RealVectorStateSpace::StateType& st, unsigned int i) { return st[i]; })
        .def("__setitem__", [](ompl::base::RealVectorStateSpace::StateType& st, unsigned int i, double v) { st[i] = v; });

    // Bind RealVectorStateSpace
    nb::class_<ompl::base::RealVectorStateSpace, ompl::base::StateSpace>(m, "RealVectorStateSpace")
        .def(nb::init<unsigned int>())
        .def("addDimension", nb::overload_cast<double, double>(&ompl::base::RealVectorStateSpace::addDimension))
        .def("addDimensionNamed", nb::overload_cast<const std::string&, double, double>(&ompl::base::RealVectorStateSpace::addDimension))
        .def("setBounds", nb::overload_cast<double, double>(&ompl::base::RealVectorStateSpace::setBounds))
        .def("getDimension", &ompl::base::RealVectorStateSpace::getDimension)
        .def("getDimensionName", &ompl::base::RealVectorStateSpace::getDimensionName)
        .def("getDimensionIndex", &ompl::base::RealVectorStateSpace::getDimensionIndex)
        .def("setDimensionName", &ompl::base::RealVectorStateSpace::setDimensionName)
        .def("getMaximumExtent", &ompl::base::RealVectorStateSpace::getMaximumExtent)
        .def("getMeasure", &ompl::base::RealVectorStateSpace::getMeasure)
        .def("enforceBounds", &ompl::base::RealVectorStateSpace::enforceBounds)
        .def("satisfiesBounds", &ompl::base::RealVectorStateSpace::satisfiesBounds)
        .def("distance", &ompl::base::RealVectorStateSpace::distance);
}