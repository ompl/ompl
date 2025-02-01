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
        .def("AddDimension", nb::overload_cast<double, double>(&ompl::base::RealVectorStateSpace::addDimension))
        .def("AddDimensionNamed", nb::overload_cast<const std::string&, double, double>(&ompl::base::RealVectorStateSpace::addDimension))
        .def("SetBounds", nb::overload_cast<double, double>(&ompl::base::RealVectorStateSpace::setBounds))
        .def("GetDimension", &ompl::base::RealVectorStateSpace::getDimension)
        .def("GetDimensionName", &ompl::base::RealVectorStateSpace::getDimensionName)
        .def("GetDimensionIndex", &ompl::base::RealVectorStateSpace::getDimensionIndex)
        .def("SetDimensionName", &ompl::base::RealVectorStateSpace::setDimensionName)
        .def("GetMaximumExtent", &ompl::base::RealVectorStateSpace::getMaximumExtent)
        .def("GetMeasure", &ompl::base::RealVectorStateSpace::getMeasure)
        .def("EnforceBounds", &ompl::base::RealVectorStateSpace::enforceBounds)
        .def("SatisfiesBounds", &ompl::base::RealVectorStateSpace::satisfiesBounds)
        .def("Distance", &ompl::base::RealVectorStateSpace::distance);
}