#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>

#include "ompl/base/PrecomputedStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::init_PrecomputedStateSampler(nb::module_ &m)
{
    nb::class_<ob::PrecomputedStateSampler, ob::StateSampler>(m, "PrecomputedStateSampler")
        .def(nb::init<const ob::StateSpace *, const std::vector<const ob::State *> &>(), nb::arg("space"),
             nb::arg("states"))
        .def(nb::init<const ob::StateSpace *, const std::vector<const ob::State *> &, std::size_t, std::size_t>(),
             nb::arg("space"), nb::arg("states"), nb::arg("minStateIndex"), nb::arg("maxStateIndex"))
        .def("sampleUniform", &ob::PrecomputedStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::PrecomputedStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::PrecomputedStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));
}
