#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/trampoline.h>

#include "ompl/base/samplers/InformedStateSampler.h"
#include "ompl/base/StateSampler.h"
#include "base/samplers/OwnerRef.h"
#include "../init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace gc = ompl::binding::gc;

namespace
{
    // OMPL keeps the cost function private, so the callable lives here where the collector can see it.
    struct PyInformedStateSampler : ob::InformedStateSampler
    {
        nb::object costFunc;

        PyInformedStateSampler(const ob::ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls, nb::object fn)
          : ob::InformedStateSampler(probDefn, maxNumberCalls, [this] { return bestCost(); }), costFunc(std::move(fn))
        {
        }

        PyInformedStateSampler(const ob::ProblemDefinitionPtr &probDefn, nb::object fn,
                               const ob::InformedSamplerPtr &infSampler)
          : ob::InformedStateSampler(probDefn, [this] { return bestCost(); }, infSampler), costFunc(std::move(fn))
        {
        }

        PyInformedStateSampler(const PyInformedStateSampler &) = delete;
        PyInformedStateSampler &operator=(const PyInformedStateSampler &) = delete;

        ob::Cost bestCost() const
        {
            nb::gil_scoped_acquire gil;
            if (!costFunc.is_valid())
                throw std::runtime_error("InformedStateSampler: the cost function is no longer set");
            return nb::cast<ob::Cost>(costFunc());
        }
    };

    PyInformedStateSampler *fromInstance(PyObject *self)
    {
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return nullptr;
        return dynamic_cast<PyInformedStateSampler *>(nb::inst_ptr<ob::InformedStateSampler>(self));
    }

    int informedTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        PyInformedStateSampler *sampler = fromInstance(self);
        if (sampler == nullptr)
            return 0;
        if (sampler->costFunc.is_valid())
            Py_VISIT(sampler->costFunc.ptr());
        if (const auto *ref = dynamic_cast<const ompl::binding::OwnerRef *>(sampler))
            return gc::visitHeld(ref->owner, visit, arg);
        return 0;
    }

    // The informed sampler in OwnerRef stays: dropping it would leave OMPL's non-owning alias dangling.
    int informedClear(PyObject *self)
    {
        if (PyInformedStateSampler *sampler = fromInstance(self))
            sampler->costFunc.reset();
        return 0;
    }

    PyType_Slot informedSlots[] = {{Py_tp_traverse, (void *)informedTraverse},
                                   {Py_tp_clear, (void *)informedClear},
                                   {0, 0}};
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

    nb::class_<ob::InformedStateSampler, ob::StateSampler, WithOwner<PyInformedStateSampler>>(
        m, "InformedStateSampler", nb::type_slots(informedSlots))
        .def(
            "__init__",
            [](ob::InformedStateSampler *self, const ob::ProblemDefinitionPtr &probDefn, unsigned int maxNumberCalls,
               nb::callable costFunc)
            {
                if (!probDefn->hasOptimizationObjective())
                    throw nb::value_error("InformedStateSampler: the problem definition has no optimization objective");
                new (self) WithOwner<PyInformedStateSampler>(nullptr, probDefn, maxNumberCalls, std::move(costFunc));
            },
            nb::arg("probDefn"), nb::arg("maxNumberCalls"), nb::arg("costFunc"))
        .def(
            "__init__",
            [](ob::InformedStateSampler *self, const ob::ProblemDefinitionPtr &probDefn, nb::callable costFunc,
               const ob::InformedSamplerPtr &infSampler)
            {
                // OMPL gets a non-owning alias so that the owning reference has a single, reportable holder.
                ob::InformedSamplerPtr borrowed(ob::InformedSamplerPtr(), infSampler.get());
                new (self) WithOwner<PyInformedStateSampler>(infSampler, probDefn, std::move(costFunc), borrowed);
            },
            nb::arg("probDefn"), nb::arg("costFunc"), nb::arg("infSampler"))
        .def("sampleUniform", &ob::InformedStateSampler::sampleUniform, nb::arg("statePtr"))
        .def("sampleUniformNear", &ob::InformedStateSampler::sampleUniformNear, nb::arg("statePtr"), nb::arg("near"),
             nb::arg("distance"))
        .def("sampleGaussian", &ob::InformedStateSampler::sampleGaussian, nb::arg("statePtr"), nb::arg("mean"),
             nb::arg("stdDev"));
}
