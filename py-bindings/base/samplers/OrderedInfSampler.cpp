#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/samplers/informed/OrderedInfSampler.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_OrderedInfSampler(nb::module_ &m)
{
    nb::class_<ob::OrderedInfSampler, ob::InformedSampler>(m, "OrderedInfSampler")
        .def(nb::init<const ob::InformedSamplerPtr &, unsigned int>(), nb::arg("infSampler"), nb::arg("batchSize"))
        .def("sampleUniform",
             nb::overload_cast<ob::State *, const ob::Cost &>(&ob::OrderedInfSampler::sampleUniform),
             nb::arg("statePtr"), nb::arg("maxCost"))
        .def("sampleUniform",
             nb::overload_cast<ob::State *, const ob::Cost &, const ob::Cost &>(
                 &ob::OrderedInfSampler::sampleUniform),
             nb::arg("statePtr"), nb::arg("minCost"), nb::arg("maxCost"))
        .def("hasInformedMeasure", &ob::OrderedInfSampler::hasInformedMeasure)
        .def("getInformedMeasure",
             nb::overload_cast<const ob::Cost &>(&ob::OrderedInfSampler::getInformedMeasure, nb::const_),
             nb::arg("currentCost"))
        .def("getInformedMeasure",
             nb::overload_cast<const ob::Cost &, const ob::Cost &>(
                 static_cast<double (ob::InformedSampler::*)(const ob::Cost &, const ob::Cost &) const>(
                     &ob::InformedSampler::getInformedMeasure),
                 nb::const_),
             nb::arg("minCost"), nb::arg("maxCost"));
}
