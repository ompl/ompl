#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <iostream>

#include "ompl/base/ScopedState.h"

// #include "../init.hh"

namespace nb = nanobind;

// Template helper to bind a ScopedState specialization.
template <typename StateSpaceType>
auto bind_scoped_state_template(nb::module_ &submod, const char *pyClassName, const char *description)
{
    nb::class_<ompl::base::ScopedState<StateSpaceType>> scopedState(submod, pyClassName, description);
    scopedState
        // Constructors:
        .def(nb::init<const std::shared_ptr<ompl::base::SpaceInformation> &>(), "Initialize the ScopedState with "
                                                                                "SpaceInformation")
        .def(nb::init<std::shared_ptr<ompl::base::StateSpace>>(), "Initialize the ScopedState with a StateSpace")
        .def(nb::init<const ompl::base::ScopedState<StateSpaceType> &>(), "Copy constructor")
        // Member functions:
        .def("random", &ompl::base::ScopedState<StateSpaceType>::random, "Set state to a random value")
        .def("enforceBounds", &ompl::base::ScopedState<StateSpaceType>::enforceBounds,
             "Enforce the bounds on this state")
        .def("satisfiesBounds", &ompl::base::ScopedState<StateSpaceType>::satisfiesBounds,
             "Check if the state satisfies its bounds")
        .def("getSpace", &ompl::base::ScopedState<StateSpaceType>::getSpace, nb::rv_policy::reference_internal,
             "Return the associated state space")
        .def("reals", &ompl::base::ScopedState<StateSpaceType>::reals, "Return the state's real values as a vector")
        .def(
            "print", [](const ompl::base::ScopedState<StateSpaceType> &self) { self.print(std::cout); },
            "Print the state to standard output")
        .def(
            "distance",
            [](const ompl::base::ScopedState<StateSpaceType> &self,
               const ompl::base::ScopedState<StateSpaceType> &other) -> double { return self.distance(other); },
            "Compute the distance to another state")
        .def(
            "__eq__",
            [](const ompl::base::ScopedState<StateSpaceType> &a, const ompl::base::ScopedState<StateSpaceType> &b)
            { return a == b; }, "Equality operator")
        .def(
            "__ne__",
            [](const ompl::base::ScopedState<StateSpaceType> &a, const ompl::base::ScopedState<StateSpaceType> &b)
            { return a != b; }, "Inequality operator")
        .def("get",
             static_cast<typename ompl::base::ScopedState<StateSpaceType>::StateType *(
                 ompl::base::ScopedState<StateSpaceType>::*)()>(&ompl::base::ScopedState<StateSpaceType>::get),
             nb::rv_policy::reference_internal, "Return a pointer to the contained state");
    return scopedState;
}
