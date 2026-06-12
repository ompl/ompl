#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include "ompl/base/samplers/informed/PathLengthDirectInfSampler.h"
#include "ompl/base/samplers/InformedStateSampler.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

void ompl::binding::base::initSamplers_PathLengthDirectInfSampler(nb::module_ &m)
{
    nb::class_<ob::PathLengthDirectInfSampler, ob::InformedSampler>(m, "PathLengthDirectInfSampler")
        .def(nb::init<const ob::ProblemDefinitionPtr &, unsigned int>(), nb::arg("probDefn"),
             nb::arg("maxNumberCalls"))
        .def("sampleUniform",
             nb::overload_cast<ob::State *, const ob::Cost &>(&ob::PathLengthDirectInfSampler::sampleUniform),
             nb::arg("statePtr"), nb::arg("maxCost"))
        .def("sampleUniform",
             nb::overload_cast<ob::State *, const ob::Cost &, const ob::Cost &>(
                 &ob::PathLengthDirectInfSampler::sampleUniform),
             nb::arg("statePtr"), nb::arg("minCost"), nb::arg("maxCost"))
        .def("hasInformedMeasure", &ob::PathLengthDirectInfSampler::hasInformedMeasure)
        .def("getInformedMeasure",
             nb::overload_cast<const ob::Cost &>(&ob::PathLengthDirectInfSampler::getInformedMeasure, nb::const_),
             nb::arg("currentCost"))
        .def("getInformedMeasure",
             nb::overload_cast<const ob::Cost &, const ob::Cost &>(
                 static_cast<double (ob::InformedSampler::*)(const ob::Cost &, const ob::Cost &) const>(
                     &ob::InformedSampler::getInformedMeasure),
                 nb::const_),
             nb::arg("minCost"), nb::arg("maxCost"))
        .def("heuristicSolnCost", &ob::PathLengthDirectInfSampler::heuristicSolnCost, nb::arg("statePtr"));
}
