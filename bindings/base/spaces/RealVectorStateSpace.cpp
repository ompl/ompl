#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/StateSpace.h"
#include "ompl/base/State.h"
#include "ompl/base/ScopedState.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "../init.hh"

namespace nb = nanobind;

void ompl::binding::base::initSpaces_RealVectorStateSpace(nb::module_& m)
{
    nb::class_<ompl::base::RealVectorStateSpace::StateType>(m, "RealVectorStateType")
        .def("__getitem__", [](const ompl::base::RealVectorStateSpace::StateType &s, unsigned int i) { return s[i]; })
        .def("__setitem__", [](ompl::base::RealVectorStateSpace::StateType &s, unsigned int i, double value) { s[i] = value; });

    auto rvSub = m.def_submodule("realvector");
    nb::class_<ompl::base::ScopedState<ompl::base::RealVectorStateSpace>>(rvSub, "State")
        .def(nb::init<const std::shared_ptr<ompl::base::SpaceInformation>&>())
        .def(nb::init<std::shared_ptr<ompl::base::StateSpace>>())
        .def("random", &ompl::base::ScopedState<ompl::base::RealVectorStateSpace>::random);

    nb::class_<ompl::base::RealVectorStateSpace, ompl::base::StateSpace>(m, "_RealVectorStateSpace")
        .def(nb::init<unsigned int>(), nb::arg("dim") = 0)
        .def("addDimension", nb::overload_cast<double, double>(&ompl::base::RealVectorStateSpace::addDimension), nb::arg("minBound"), nb::arg("maxBound"))
        .def("addDimension", nb::overload_cast<const std::string &, double, double>(&ompl::base::RealVectorStateSpace::addDimension), nb::arg("name"), nb::arg("minBound"), nb::arg("maxBound"))
        .def("setBounds", nb::overload_cast<const ompl::base::RealVectorBounds&>(&ompl::base::RealVectorStateSpace::setBounds))
        .def("setBounds", nb::overload_cast<double, double>(&ompl::base::RealVectorStateSpace::setBounds))
        .def("getBounds", &ompl::base::RealVectorStateSpace::getBounds, nb::rv_policy::reference)
        .def("getDimension", &ompl::base::RealVectorStateSpace::getDimension)
        .def("getDimensionName", &ompl::base::RealVectorStateSpace::getDimensionName)
        .def("getDimensionIndex", &ompl::base::RealVectorStateSpace::getDimensionIndex)
        .def("setDimensionName", &ompl::base::RealVectorStateSpace::setDimensionName)
        .def("allocState", &ompl::base::RealVectorStateSpace::allocState)
        .def("freeState", &ompl::base::RealVectorStateSpace::freeState)
        .def("registerProjections", &ompl::base::RealVectorStateSpace::registerProjections);

    m.def("RealVectorStateSpace", [](unsigned int dim) { return std::make_shared<ompl::base::RealVectorStateSpace>(dim); }, nb::rv_policy::reference);
}