#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/pair.h>

#include "ompl/base/ConstrainedSpaceInformation.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_ConstrainedSpaceInformation(nb::module_ &m)
{
    nb::class_<ob::ConstrainedValidStateSampler, ob::ValidStateSampler>(m, "ConstrainedValidStateSampler");

    nb::class_<ob::ConstrainedSpaceInformation, ob::SpaceInformation>(m, "ConstrainedSpaceInformation")
        .def(nb::init<ob::StateSpacePtr>(), nb::arg("space"))
        .def(
            "getMotionStates",
            [](const ob::ConstrainedSpaceInformation &si, const ob::State *s1, const ob::State *s2, unsigned int count,
               bool endpoints)
            {
                std::vector<ob::State *> states;
                unsigned int added = si.getMotionStates(s1, s2, states, count, endpoints, true);
                std::vector<std::shared_ptr<ob::State>> owned;
                owned.reserve(added);
                auto space = si.getStateSpace();
                for (unsigned int i = 0; i < added; ++i)
                    owned.emplace_back(states[i], [space](ob::State *s) { space->freeState(s); });
                return owned;
            },
            nb::arg("s1"), nb::arg("s2"), nb::arg("count"), nb::arg("endpoints") = true);

    nb::class_<ob::TangentBundleSpaceInformation, ob::ConstrainedSpaceInformation>(m, "TangentBundleSpaceInformation")
        .def(nb::init<ob::StateSpacePtr>(), nb::arg("space"));
}
