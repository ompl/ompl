#include <nanobind/nanobind.h>
#include <nanobind/stl/function.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>

#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;

namespace
{
    ob::InformedStateSampler::GetCurrentCostFunc wrapCostFunc(ob::InformedStateSampler::GetCurrentCostFunc fn)
    {
        return [fn = std::move(fn)]() -> ob::Cost
        {
            nb::gil_scoped_acquire gil;
            return fn();
        };
    }
}  // namespace

void ompl::binding::base::initSamplers_InformedStateSampler(nb::module_ &m)
{
    struct PyInformedSampler : ob::InformedSampler
    {
        NB_TRAMPOLINE(ob::InformedSampler, 5);

        bool sampleUniform(ob::State *statePtr, const ob::Cost &maxCost) override
        {
            NB_OVERRIDE_PURE(sampleUniform, statePtr, maxCost);
        }

        bool sampleUniform(ob::State *statePtr, const ob::Cost &minCost, const ob::Cost &maxCost) override
        {
            NB_OVERRIDE_PURE(sampleUniform, statePtr, minCost, maxCost);
        }

        bool hasInformedMeasure() const override
        {
            NB_OVERRIDE_PURE(hasInformedMeasure);
        }

        double getInformedMeasure(const ob::Cost &currentCost) const override
        {
            NB_OVERRIDE_PURE(getInformedMeasure, currentCost);
        }

        double getInformedMeasure(const ob::Cost &minCost, const ob::Cost &maxCost) const override
        {
            NB_OVERRIDE(getInformedMeasure, minCost, maxCost);
        }
    };

    nb::class_<ob::InformedSampler, PyInformedSampler>(m, "InformedSampler")
        .def(nb::init<const ob::ProblemDefinitionPtr &, unsigned int>(), nb::arg("probDefn"), nb::arg("maxNumberCalls"))
        .def("sampleUniform", nb::overload_cast<ob::State *, const ob::Cost &>(&ob::InformedSampler::sampleUniform),
             nb::arg("statePtr"), nb::arg("maxCost"))
        .def("sampleUniform",
             nb::overload_cast<ob::State *, const ob::Cost &, const ob::Cost &>(&ob::InformedSampler::sampleUniform),
             nb::arg("statePtr"), nb::arg("minCost"), nb::arg("maxCost"))
        .def("hasInformedMeasure", &ob::InformedSampler::hasInformedMeasure)
        .def("getInformedMeasure",
             nb::overload_cast<const ob::Cost &>(&ob::InformedSampler::getInformedMeasure, nb::const_),
             nb::arg("currentCost"))
        .def(
            "getInformedMeasure",
            nb::overload_cast<const ob::Cost &, const ob::Cost &>(&ob::InformedSampler::getInformedMeasure, nb::const_),
            nb::arg("minCost"), nb::arg("maxCost"))
        .def("heuristicSolnCost", &ob::InformedSampler::heuristicSolnCost, nb::arg("statePtr"))
        .def("getProblemDefn", &ob::InformedSampler::getProblemDefn)
        .def("getMaxNumberOfIters", &ob::InformedSampler::getMaxNumberOfIters);

    nb::class_<ob::InformedStateSampler, ob::StateSampler>(m, "InformedStateSampler")
        .def(
            "__init__",
            [](ob::InformedStateSampler *self, const ob::ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls,
               ob::InformedStateSampler::GetCurrentCostFunc costFunc)
            { new (self) ob::InformedStateSampler(probDefn, maxNumberCalls, wrapCostFunc(std::move(costFunc))); },
            nb::arg("probDefn"), nb::arg("maxNumberCalls"), nb::arg("costFunc"))
        .def(
            "__init__",
            [](ob::InformedStateSampler *self, const ob::ProblemDefinitionPtr &probDefn,
               ob::InformedStateSampler::GetCurrentCostFunc costFunc, const ob::InformedSamplerPtr &infSampler)
            { new (self) ob::InformedStateSampler(probDefn, wrapCostFunc(std::move(costFunc)), infSampler); },
            nb::arg("probDefn"), nb::arg("costFunc"), nb::arg("infSampler"))
        .def("sampleUniform", &ob::InformedStateSampler::sampleUniform, nb::arg("statePtr"))
        .def("sampleUniformNear", &ob::InformedStateSampler::sampleUniformNear, nb::arg("statePtr"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::InformedStateSampler::sampleGaussian, nb::arg("statePtr"), nb::arg("mean"),
             nb::arg("stdDev"));
}
