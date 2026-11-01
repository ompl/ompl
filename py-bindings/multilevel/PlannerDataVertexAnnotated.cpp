#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/multilevel/datastructures/PlannerDataVertexAnnotated.h>
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace om = ompl::multilevel;

void ompl::binding::multilevel::init_PlannerDataVertexAnnotated(nb::module_ &m)
{
    nb::class_<om::PlannerDataVertexAnnotated, ob::PlannerDataVertex>(m, "PlannerDataVertexAnnotated")
        // The vertex keeps a pointer to the base state but does not own it.
        .def(nb::init<const ob::State *>(), nb::arg("state"), nb::keep_alive<1, 2>())
        .def("setLevel", &om::PlannerDataVertexAnnotated::setLevel, nb::arg("level"))
        .def("getLevel", &om::PlannerDataVertexAnnotated::getLevel)
        .def("setMaxLevel", &om::PlannerDataVertexAnnotated::setMaxLevel, nb::arg("level"))
        .def("getMaxLevel", &om::PlannerDataVertexAnnotated::getMaxLevel)
        .def("setComponent", &om::PlannerDataVertexAnnotated::setComponent, nb::arg("component"))
        .def("getComponent", &om::PlannerDataVertexAnnotated::getComponent)
        // The vertex frees the total state, so it gets its own copy rather than the caller's state.
        .def(
            "setTotalState",
            [](om::PlannerDataVertexAnnotated &v, const ob::State *state, const ob::SpaceInformationPtr &si)
            {
                if (!si)
                    throw nb::value_error("setTotalState() needs the SpaceInformation of the total space");
                v.setTotalState(si->cloneState(state), si);
            },
            nb::arg("state"), nb::arg("si"))
        .def("setBaseState", &om::PlannerDataVertexAnnotated::setBaseState, nb::arg("state"), nb::keep_alive<1, 2>())
        .def("getState", &om::PlannerDataVertexAnnotated::getState, nb::rv_policy::reference_internal)
        .def("getStateNonConst", &om::PlannerDataVertexAnnotated::getStateNonConst, nb::rv_policy::reference_internal)
        .def("getBaseState", &om::PlannerDataVertexAnnotated::getBaseState, nb::rv_policy::reference_internal)
        .def("getSpaceInformationPtr", &om::PlannerDataVertexAnnotated::getSpaceInformationPtr)
        .def("clone", &om::PlannerDataVertexAnnotated::clone);
}
