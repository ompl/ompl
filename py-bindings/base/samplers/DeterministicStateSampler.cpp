#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <string>

#include "ompl/base/samplers/DeterministicStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"
#include "base/samplers/OwnerRef.h"
#include "../validation.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

namespace
{
    using SamplerType = ob::DeterministicStateSampler::DeterministicSamplerType;
    using SequencePtr = std::shared_ptr<ob::DeterministicSequence>;

    // copyFromReals does not check the sample's length, so check it before every copy.
    struct CheckedDeterministicStateSampler : ob::DeterministicStateSampler
    {
        using ob::DeterministicStateSampler::DeterministicStateSampler;

        void sampleUniform(ob::State *state) override
        {
            std::vector<double> sample = sequence_ptr_->sample();
            ompl::binding::base::checkRealsSize(*space_, sample, "DeterministicStateSampler.sampleUniform");
            space_->copyFromReals(state, sample);
        }
    };

    void checkSequence(const SequencePtr &sequence, unsigned int expected, const char *sampler)
    {
        if (sequence->dimensions_ < expected)
            throw nb::value_error((std::string(sampler) + ": the sequence has " +
                                   std::to_string(sequence->dimensions_) + " dimension(s), but the sampler needs " +
                                   std::to_string(expected))
                                      .c_str());
    }
}  // namespace

void ompl::binding::base::initSamplers_DeterministicStateSampler(nb::module_ &m)
{
    nb::enum_<SamplerType>(m, "DeterministicSamplerType").value("HALTON", ob::DeterministicStateSampler::HALTON);

    nb::class_<ob::DeterministicStateSampler, ob::StateSampler, WithOwner<CheckedDeterministicStateSampler>>(
        m, "DeterministicStateSampler", nb::type_slots(ownerSlots<ob::DeterministicStateSampler>))
        .def("__init__",
             initWithOwnerAs<ob::DeterministicStateSampler, CheckedDeterministicStateSampler, ob::StateSpace,
                             SamplerType>(),
             nb::arg("space"), nb::arg("type") = ob::DeterministicStateSampler::HALTON)
        .def(
            "__init__", [](ob::DeterministicStateSampler *self, const ob::StateSpacePtr &space, SequencePtr sequence)
            { new (self) WithOwner<CheckedDeterministicStateSampler>(space, space.get(), std::move(sequence)); },
            nb::arg("space"), nb::arg("sequence"))
        .def("sampleUniform", &ob::DeterministicStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::DeterministicStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::DeterministicStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));

    nb::class_<ob::SO2DeterministicStateSampler, ob::DeterministicStateSampler,
               WithOwner<ob::SO2DeterministicStateSampler>>(
        m, "SO2DeterministicStateSampler", nb::type_slots(ownerSlots<ob::SO2DeterministicStateSampler>))
        .def("__init__", initWithOwner<ob::SO2DeterministicStateSampler, ob::StateSpace, SamplerType>(),
             nb::arg("space"), nb::arg("type") = ob::DeterministicStateSampler::HALTON)
        .def(
            "__init__",
            [](ob::SO2DeterministicStateSampler *self, const ob::StateSpacePtr &space, SequencePtr sequence)
            {
                checkSequence(sequence, 1, "SO2DeterministicStateSampler");
                new (self) WithOwner<ob::SO2DeterministicStateSampler>(space, space.get(), std::move(sequence));
            },
            nb::arg("space"), nb::arg("sequence"))
        .def("sampleUniform", &ob::SO2DeterministicStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::SO2DeterministicStateSampler::sampleUniformNear, nb::arg("state"),
             nb::arg("near"), nb::arg("distance"))
        .def("sampleGaussian", &ob::SO2DeterministicStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));

    nb::class_<ob::RealVectorDeterministicStateSampler, ob::DeterministicStateSampler,
               WithOwner<ob::RealVectorDeterministicStateSampler>>(
        m, "RealVectorDeterministicStateSampler", nb::type_slots(ownerSlots<ob::RealVectorDeterministicStateSampler>))
        .def("__init__", initWithOwner<ob::RealVectorDeterministicStateSampler, ob::StateSpace, SamplerType>(),
             nb::arg("space"), nb::arg("type") = ob::DeterministicStateSampler::HALTON)
        .def(
            "__init__",
            [](ob::RealVectorDeterministicStateSampler *self, const ob::StateSpacePtr &space, SequencePtr sequence,
               bool stretch)
            {
                checkSequence(sequence, space->getDimension(), "RealVectorDeterministicStateSampler");
                new (self) WithOwner<ob::RealVectorDeterministicStateSampler>(space, space.get(), std::move(sequence),
                                                                              stretch);
            },
            nb::arg("space"), nb::arg("sequence"), nb::arg("stretch") = true)
        .def("sampleUniform", &ob::RealVectorDeterministicStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::RealVectorDeterministicStateSampler::sampleUniformNear, nb::arg("state"),
             nb::arg("near"), nb::arg("distance"))
        .def("sampleGaussian", &ob::RealVectorDeterministicStateSampler::sampleGaussian, nb::arg("state"),
             nb::arg("mean"), nb::arg("stdDev"));

    nb::class_<ob::SE2DeterministicStateSampler, ob::DeterministicStateSampler,
               WithOwner<ob::SE2DeterministicStateSampler>>(
        m, "SE2DeterministicStateSampler", nb::type_slots(ownerSlots<ob::SE2DeterministicStateSampler>))
        .def("__init__", initWithOwner<ob::SE2DeterministicStateSampler, ob::StateSpace, SamplerType>(),
             nb::arg("space"), nb::arg("type") = ob::DeterministicStateSampler::HALTON)
        .def(
            "__init__",
            [](ob::SE2DeterministicStateSampler *self, const ob::StateSpacePtr &space, SequencePtr sequence,
               bool stretchRv, bool stretchSo2)
            {
                checkSequence(sequence, 3, "SE2DeterministicStateSampler");
                new (self) WithOwner<ob::SE2DeterministicStateSampler>(space, space.get(), std::move(sequence),
                                                                       stretchRv, stretchSo2);
            },
            nb::arg("space"), nb::arg("sequence"), nb::arg("stretch_rv") = true, nb::arg("stretch_so2") = true)
        .def("sampleUniform", &ob::SE2DeterministicStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::SE2DeterministicStateSampler::sampleUniformNear, nb::arg("state"),
             nb::arg("near"), nb::arg("distance"))
        .def("sampleGaussian", &ob::SE2DeterministicStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));
}
