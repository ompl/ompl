#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_StateSampler(nb::module_& m)
{
    nb::class_<ompl::base::StateSampler>(m, "StateSampler");
    nb::class_<ompl::base::CompoundStateSampler, ompl::base::StateSampler>(m, "CompoundStateSampler")
        .def(nb::init<const ompl::base::StateSpace *>(), nb::arg("space"),
             "Construct a CompoundStateSampler for the given state space.")
        .def("addSampler", &ompl::base::CompoundStateSampler::addSampler)
        .def("sampleUniform", &ompl::base::CompoundStateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::CompoundStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::CompoundStateSampler::sampleGaussian);
    nb::class_<ompl::base::SubspaceStateSampler, ompl::base::StateSampler>(m, "SubspaceStateSampler")
        .def(nb::init<const ompl::base::StateSpace *, const ompl::base::StateSpace *, double>(), nb::arg("space"), nb::arg("subspace"), nb::arg("weight"),
             "Construct a SubspaceStateSampler for the given state space and subspace.")
        .def("sampleUniform", &ompl::base::SubspaceStateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::SubspaceStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::SubspaceStateSampler::sampleGaussian);
}
