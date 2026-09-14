#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/samplers/DeterministicStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "ompl/base/StateSpace.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_DeterministicStateSampler(nb::module_ &m)
{
    nb::enum_<ob::DeterministicStateSampler::DeterministicSamplerType>(m, "DeterministicSamplerType")
        .value("HALTON", ob::DeterministicStateSampler::HALTON);

    nb::class_<ob::DeterministicStateSampler, ob::StateSampler>(m, "DeterministicStateSampler")
        .def(nb::init<const ob::StateSpace *, ob::DeterministicStateSampler::DeterministicSamplerType>(),
             nb::arg("space"), nb::arg("type") = ob::DeterministicStateSampler::HALTON)
        .def(nb::init<const ob::StateSpace *, std::shared_ptr<ob::DeterministicSequence>>(), nb::arg("space"),
             nb::arg("sequence"))
        .def("sampleUniform", &ob::DeterministicStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::DeterministicStateSampler::sampleUniformNear, nb::arg("state"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::DeterministicStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));

    nb::class_<ob::SO2DeterministicStateSampler, ob::DeterministicStateSampler>(m, "SO2DeterministicStateSampler")
        .def(nb::init<const ob::StateSpace *, ob::DeterministicStateSampler::DeterministicSamplerType>(),
             nb::arg("space"), nb::arg("type") = ob::DeterministicStateSampler::HALTON)
        .def(nb::init<const ob::StateSpace *, std::shared_ptr<ob::DeterministicSequence>>(), nb::arg("space"),
             nb::arg("sequence"))
        .def("sampleUniform", &ob::SO2DeterministicStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::SO2DeterministicStateSampler::sampleUniformNear, nb::arg("state"),
             nb::arg("near"), nb::arg("distance"))
        .def("sampleGaussian", &ob::SO2DeterministicStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));

    nb::class_<ob::RealVectorDeterministicStateSampler, ob::DeterministicStateSampler>(
        m, "RealVectorDeterministicStateSampler")
        .def(nb::init<const ob::StateSpace *, ob::DeterministicStateSampler::DeterministicSamplerType>(),
             nb::arg("space"), nb::arg("type") = ob::DeterministicStateSampler::HALTON)
        .def(nb::init<const ob::StateSpace *, std::shared_ptr<ob::DeterministicSequence>, bool>(), nb::arg("space"),
             nb::arg("sequence"), nb::arg("stretch") = true)
        .def("sampleUniform", &ob::RealVectorDeterministicStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::RealVectorDeterministicStateSampler::sampleUniformNear, nb::arg("state"),
             nb::arg("near"), nb::arg("distance"))
        .def("sampleGaussian", &ob::RealVectorDeterministicStateSampler::sampleGaussian, nb::arg("state"),
             nb::arg("mean"), nb::arg("stdDev"));

    nb::class_<ob::SE2DeterministicStateSampler, ob::DeterministicStateSampler>(m, "SE2DeterministicStateSampler")
        .def(nb::init<const ob::StateSpace *, ob::DeterministicStateSampler::DeterministicSamplerType>(),
             nb::arg("space"), nb::arg("type") = ob::DeterministicStateSampler::HALTON)
        .def(nb::init<const ob::StateSpace *, std::shared_ptr<ob::DeterministicSequence>, bool, bool>(),
             nb::arg("space"), nb::arg("sequence"), nb::arg("stretch_rv") = true, nb::arg("stretch_so2") = true)
        .def("sampleUniform", &ob::SE2DeterministicStateSampler::sampleUniform, nb::arg("state"))
        .def("sampleUniformNear", &ob::SE2DeterministicStateSampler::sampleUniformNear, nb::arg("state"),
             nb::arg("near"), nb::arg("distance"))
        .def("sampleGaussian", &ob::SE2DeterministicStateSampler::sampleGaussian, nb::arg("state"), nb::arg("mean"),
             nb::arg("stdDev"));
}
