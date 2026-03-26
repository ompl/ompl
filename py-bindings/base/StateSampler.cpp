#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"
#include "init.h"

namespace nb = nanobind;

void ompl::binding::base::init_StateSampler(nb::module_ &m)
{
    struct PyStateSampler : ompl::base::StateSampler
    {
        NB_TRAMPOLINE(ompl::base::StateSampler, 3);

        void sampleUniform(ompl::base::State *state) override
        {
            NB_OVERRIDE_PURE(sampleUniform, state);
        }

        void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override
        {
            NB_OVERRIDE_PURE(sampleUniformNear, state, near, distance);
        }

        void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override
        {
            NB_OVERRIDE_PURE(sampleGaussian, state, mean, stdDev);
        }
    };

    nb::class_<ompl::base::StateSampler, PyStateSampler>(m, "StateSampler")
        .def(nb::init<const ompl::base::StateSpace *>(), nb::arg("space"))
        .def("sampleUniform", &ompl::base::StateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::StateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::StateSampler::sampleGaussian);
    nb::class_<ompl::base::CompoundStateSampler, ompl::base::StateSampler>(m, "CompoundStateSampler")
        .def(nb::init<const ompl::base::StateSpace *>(), nb::arg("space"),
             "Construct a CompoundStateSampler for the given state space.")
        .def("addSampler", &ompl::base::CompoundStateSampler::addSampler)
        .def("sampleUniform", &ompl::base::CompoundStateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::CompoundStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::CompoundStateSampler::sampleGaussian);
    nb::class_<ompl::base::SubspaceStateSampler, ompl::base::StateSampler>(m, "SubspaceStateSampler")
        .def(nb::init<const ompl::base::StateSpace *, const ompl::base::StateSpace *, double>(), nb::arg("space"),
             nb::arg("subspace"), nb::arg("weight"),
             "Construct a SubspaceStateSampler for the given state space and subspace.")
        .def("sampleUniform", &ompl::base::SubspaceStateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::SubspaceStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::SubspaceStateSampler::sampleGaussian);
}
