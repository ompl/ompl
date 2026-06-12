#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::init_PlannerDataVertexAnnotated(nb::module_ &m)
{
    nb::class_<om::PlannerDataVertexAnnotated, ob::PlannerDataVertex>(m, "PlannerDataVertexAnnotated")
        .def(nb::init<const ob::State *>(), nb::arg("state"))
        .def("setLevel", &om::PlannerDataVertexAnnotated::setLevel, nb::arg("level"))
        .def("getLevel", &om::PlannerDataVertexAnnotated::getLevel)
        .def("setMaxLevel", &om::PlannerDataVertexAnnotated::setMaxLevel, nb::arg("level"))
        .def("getMaxLevel", &om::PlannerDataVertexAnnotated::getMaxLevel)
        .def("setComponent", &om::PlannerDataVertexAnnotated::setComponent, nb::arg("component"))
        .def("getComponent", &om::PlannerDataVertexAnnotated::getComponent)
        .def("setTotalState", &om::PlannerDataVertexAnnotated::setTotalState, nb::arg("state"), nb::arg("si"))
        .def("setBaseState", &om::PlannerDataVertexAnnotated::setBaseState, nb::arg("state"))
        .def("getState", &om::PlannerDataVertexAnnotated::getState, nb::rv_policy::reference)
        .def("getStateNonConst", &om::PlannerDataVertexAnnotated::getStateNonConst, nb::rv_policy::reference)
        .def("getBaseState", &om::PlannerDataVertexAnnotated::getBaseState, nb::rv_policy::reference)
        .def("getSpaceInformationPtr", &om::PlannerDataVertexAnnotated::getSpaceInformationPtr)
        .def("clone", &om::PlannerDataVertexAnnotated::clone);
}
