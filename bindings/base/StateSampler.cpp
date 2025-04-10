#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include "ompl/base/StateSampler.h"
#include "init.hh"

namespace nb = nanobind;

void ompl::binding::base::init_StateSampler(nb::module_& m)
{
    nb::class_<ompl::base::StateSampler>(m, "StateSampler");
    nb::class_<ompl::base::CompoundStateSampler, ompl::base::StateSampler>(m, "CompoundStateSampler")
        // .def("addSampler", &ompl::base::CompoundStateSampler::addSampler)
        .def("sampleUniform", &ompl::base::CompoundStateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::CompoundStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::CompoundStateSampler::sampleGaussian);
    nb::class_<ompl::base::SubspaceStateSampler, ompl::base::StateSampler>(m, "SubspaceStateSampler")
        .def("sampleUniform", &ompl::base::SubspaceStateSampler::sampleUniform)
        .def("sampleUniformNear", &ompl::base::SubspaceStateSampler::sampleUniformNear)
        .def("sampleGaussian", &ompl::base::SubspaceStateSampler::sampleGaussian);
}
