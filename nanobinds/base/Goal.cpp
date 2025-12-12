#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/string.h>

#include <sstream>

#include "ompl/base/Goal.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_Goal(nb::module_ &m)
{
    struct PyGoal : ob::Goal
    {
        NB_TRAMPOLINE(ob::Goal, 3);
        // Pure virtual: must override in Python
        bool isSatisfied(const ob::State *st) const override
        {
            NB_OVERRIDE_PURE(isSatisfied, st);
        }

        // Optional override
        bool isStartGoalPairValid(const ob::State *start, const ob::State *goal) const override
        {
            NB_OVERRIDE(isStartGoalPairValid, start, goal);
        }

        // Optional override
        void print(std::ostream &out) const override
        {
            NB_OVERRIDE(print, out);
        }
    };

    // TODO [ob::Goal][TEST]
    nb::class_<ob::Goal, PyGoal /* <-- trampoline */>(m, "Goal")
        // constructor
        .def(nb::init<const ob::SpaceInformationPtr &>(), nb::arg("si"))
        // getters
        .def("getType", &ob::Goal::getType)
        .def("hasType", &ob::Goal::hasType, nb::arg("type"))
        .def("getSpaceInformation", &ob::Goal::getSpaceInformation, nb::rv_policy::reference_internal);
}
