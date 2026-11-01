#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/samplers/informed/RejectionInfSampler.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_RejectionInfSampler(nb::module_ &m)
{
    nb::class_<ob::RejectionInfSampler, ob::InformedSampler>(m, "RejectionInfSampler")
        .def(nb::init<const ob::ProblemDefinitionPtr &, unsigned int>(), nb::arg("probDefn"), nb::arg("maxNumberCalls"))
        .def("sampleUniform", nb::overload_cast<ob::State *, const ob::Cost &>(&ob::RejectionInfSampler::sampleUniform),
             nb::arg("statePtr"), nb::arg("maxCost"))
        .def(
            "sampleUniform",
            nb::overload_cast<ob::State *, const ob::Cost &, const ob::Cost &>(&ob::RejectionInfSampler::sampleUniform),
            nb::arg("statePtr"), nb::arg("minCost"), nb::arg("maxCost"))
        .def("hasInformedMeasure", &ob::RejectionInfSampler::hasInformedMeasure)
        .def("getInformedMeasure",
             nb::overload_cast<const ob::Cost &>(&ob::RejectionInfSampler::getInformedMeasure, nb::const_),
             nb::arg("currentCost"))
        .def("getInformedMeasure",
             nb::overload_cast<const ob::Cost &, const ob::Cost &>(&ob::RejectionInfSampler::getInformedMeasure,
                                                                   nb::const_),
             nb::arg("minCost"), nb::arg("maxCost"));
}
