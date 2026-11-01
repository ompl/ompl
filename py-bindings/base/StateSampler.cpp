#include <nanobind/nanobind.h>
#include <nanobind/trampoline.h>
#include <nanobind/stl/shared_ptr.h>
#include <memory>
#include <utility>
#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"
#include "init.h"
#include "base/samplers/OwnerRef.h"

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

    nb::class_<ompl::base::StateSampler, ompl::binding::WithOwner<PyStateSampler>>(
        m, "StateSampler", nb::type_slots(ompl::binding::ownerSlots<ompl::base::StateSampler>))
        .def("__init__",
             ompl::binding::initWithOwnerAs<ompl::base::StateSampler, PyStateSampler, ompl::base::StateSpace>(),
             nb::arg("space"))
        .def("sampleUniform", &ompl::base::StateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::StateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::StateSampler::sampleGaussian);
    nb::class_<ompl::base::CompoundStateSampler, ompl::base::StateSampler,
               ompl::binding::WithOwner<ompl::base::CompoundStateSampler>>(
        m, "CompoundStateSampler", nb::type_slots(ompl::binding::ownerSlots<ompl::base::CompoundStateSampler>))
        .def("__init__", ompl::binding::initWithOwner<ompl::base::CompoundStateSampler, ompl::base::StateSpace>(),
             nb::arg("space"), "Construct a CompoundStateSampler for the given state space.")
        .def("addSampler", &ompl::base::CompoundStateSampler::addSampler)
        .def("sampleUniform", &ompl::base::CompoundStateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::CompoundStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::CompoundStateSampler::sampleGaussian);
    nb::class_<ompl::base::SubspaceStateSampler, ompl::base::StateSampler,
               ompl::binding::WithOwner<ompl::base::SubspaceStateSampler>>(m, "SubspaceStateSampler")
        .def(
            "__init__",
            [](ompl::base::SubspaceStateSampler *self, const ompl::base::StateSpacePtr &space,
               const ompl::base::StateSpacePtr &subspace, double weight)
            {
                // A pair is not a py_deleter reference, so the collector never sees these: a cycle leaks.
                auto owners = std::make_shared<const std::pair<ompl::base::StateSpacePtr, ompl::base::StateSpacePtr>>(
                    space, subspace);
                new (self) ompl::binding::WithOwner<ompl::base::SubspaceStateSampler>(owners, space.get(),
                                                                                      subspace.get(), weight);
            },
            nb::arg("space"), nb::arg("subspace"), nb::arg("weight"),
            "Construct a SubspaceStateSampler for the given state space and subspace.")
        .def("sampleUniform", &ompl::base::SubspaceStateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::SubspaceStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::SubspaceStateSampler::sampleGaussian);
}
