#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <memory>
#include <vector>

#include "ompl/base/PrecomputedStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

namespace
{
    /// PrecomputedStateSampler only refers to the vector, the space and the states; this owns them.
    struct PyPrecomputedStateSampler : ob::PrecomputedStateSampler
    {
        PyPrecomputedStateSampler(ob::StateSpacePtr space, std::unique_ptr<std::vector<const ob::State *>> states,
                                  nb::tuple keep)
          : ob::PrecomputedStateSampler(space.get(), *states)
          , space_(std::move(space))
          , states_(std::move(states))
          , keep_(std::move(keep))
        {
        }

        PyPrecomputedStateSampler(ob::StateSpacePtr space, std::unique_ptr<std::vector<const ob::State *>> states,
                                  nb::tuple keep, std::size_t minStateIndex, std::size_t maxStateIndex)
          : ob::PrecomputedStateSampler(space.get(), *states, minStateIndex, maxStateIndex)
          , space_(std::move(space))
          , states_(std::move(states))
          , keep_(std::move(keep))
        {
        }

        ob::StateSpacePtr space_;
        std::unique_ptr<std::vector<const ob::State *>> states_;
        nb::tuple keep_;
    };

    std::unique_ptr<std::vector<const ob::State *>> statePointers(const nb::tuple &states)
    {
        auto pointers = std::make_unique<std::vector<const ob::State *>>();
        pointers->reserve(states.size());
        for (nb::handle state : states)
        {
            if (state.is_none())
                throw nb::type_error("states must not contain None");
            pointers->push_back(nb::cast<const ob::State *>(state));
        }
        return pointers;
    }
}  // namespace

void ompl::binding::base::init_PrecomputedStateSampler(nb::module_ &m)
{
    nb::class_<ob::PrecomputedStateSampler, ob::StateSampler, PyPrecomputedStateSampler>(m, "PrecomputedStateSampler")
        .def(
            "__init__",
            [](ob::PrecomputedStateSampler *self, ob::StateSpacePtr space, nb::sequence states)
            {
                nb::tuple keep(states);
                new (self) PyPrecomputedStateSampler(std::move(space), statePointers(keep), keep);
            },
            nb::arg("space"), nb::arg("states"))
        .def(
            "__init__",
            [](ob::PrecomputedStateSampler *self, ob::StateSpacePtr space, nb::sequence states,
               std::size_t minStateIndex, std::size_t maxStateIndex)
            {
                nb::tuple keep(states);
                new (self) PyPrecomputedStateSampler(std::move(space), statePointers(keep), keep, minStateIndex,
                                                     maxStateIndex);
            },
            nb::arg("space"), nb::arg("states"), nb::arg("minStateIndex"), nb::arg("maxStateIndex"))
        .def("sampleUniform", &ob::PrecomputedStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::PrecomputedStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::PrecomputedStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));
}
