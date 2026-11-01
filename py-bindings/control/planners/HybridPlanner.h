#pragma once

#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

#include "ompl/base/Planner.h"
#include "ompl/base/PlannerData.h"
#include "ompl/base/SpaceInformation.h"
#include "ompl/control/Control.h"
#include "ompl/control/SpaceInformation.h"
#include "PlannerSolve.h"
#include "PyGC.h"

namespace ompl::binding::control::hybrid
{
    namespace nb = nanobind;
    namespace gc = ompl::binding::gc;
    namespace ob = ompl::base;
    namespace oc = ompl::control;

    inline nb::handle callable(const gc::SharedRef &fn)
    {
        nb::handle h = fn.get();
        if (!h.is_valid())
            throw std::runtime_error("the Python callback is no longer set");
        return h;
    }

    // OMPL keeps the returned pointer, so a state Python allocated is copied into newState; None means filled in place.
    inline ob::State *intoNewState(const ob::SpaceInformation *si, nb::handle result, ob::State *newState)
    {
        if (result.is_none())
            return newState;
        auto *returned = nb::cast<ob::State *>(result);
        if (returned != newState)
            si->copyState(newState, returned);
        return newState;
    }

    struct DiscreteSimulator
    {
        gc::SharedRef fn;
        const ob::SpaceInformation *si;

        ob::State *operator()(ob::State *curState, const oc::Control *u, ob::State *newState) const
        {
            nb::gil_scoped_acquire gil;
            nb::object result = callable(fn)(curState, u, newState);
            return intoNewState(si, result, newState);
        }
    };

    // solve() copies continuousSimulator into continuousSimulator_, so traverse counts both copies against use_count.
    struct ContinuousSimulator
    {
        std::shared_ptr<const gc::SharedRef> fn;
        const ob::SpaceInformation *si;

        ob::State *operator()(const oc::Control *u, ob::State *curState, double tFlowMax, ob::State *newState) const
        {
            nb::gil_scoped_acquire gil;
            nb::object result = callable(*fn)(u, curState, tFlowMax, newState);
            return intoNewState(si, result, newState);
        }
    };

    // Python returns a bool, or (bool, collisionTime) to set the out-parameter.
    template <typename Motion>
    struct CollisionChecker
    {
        gc::SharedRef fn;

        bool operator()(Motion *motion, std::function<bool(Motion *)> obstacleSet, ob::State *newState,
                        double *collisionTime) const
        {
            nb::gil_scoped_acquire gil;
            nb::object obstacle =
                nb::cpp_function([set = std::move(obstacleSet)](Motion *m) { return set(m); }, nb::arg("motion"));
            nb::object result = callable(fn)(motion, obstacle, newState);
            if (!nb::isinstance<nb::tuple>(result))
                return nb::cast<bool>(result);
            auto pair = nb::borrow<nb::tuple>(result);
            if (pair.size() != 2)
                throw nb::value_error("a collision checker returns a bool or a (bool, collisionTime) pair");
            if (collisionTime != nullptr && !pair[1].is_none())
                *collisionTime = nb::cast<double>(pair[1]);
            return nb::cast<bool>(pair[0]);
        }
    };

    // The planner keeps its callbacks in protected members, which traverse and clear must reach.
    template <typename Planner>
    struct Access : Planner
    {
        using Motion = typename Planner::Motion;
        using SetFn = std::function<bool(Motion *)>;
        using DistanceFn = std::function<double(ob::State *, ob::State *)>;
        using DiscreteFn = std::function<ob::State *(ob::State *, const oc::Control *, ob::State *)>;
        using ContinuousFn = std::function<ob::State *(const oc::Control *, ob::State *, double, ob::State *)>;
        using CollisionFn = std::function<bool(Motion *, std::function<bool(Motion *)>, ob::State *, double *)>;

        static SetFn &jumpSet(Planner &p)
        {
            return p.*(&Access::jumpSet_);
        }
        static SetFn &flowSet(Planner &p)
        {
            return p.*(&Access::flowSet_);
        }
        static SetFn &unsafeSet(Planner &p)
        {
            return p.*(&Access::unsafeSet_);
        }
        static DistanceFn &distanceFunc(Planner &p)
        {
            return p.*(&Access::distanceFunc_);
        }
        static DiscreteFn &discreteSimulator(Planner &p)
        {
            return p.*(&Access::discreteSimulator_);
        }
        static ContinuousFn &continuousSimulator(Planner &p)
        {
            return p.*(&Access::continuousSimulator_);
        }
        static CollisionFn &collisionChecker(Planner &p)
        {
            return p.*(&Access::collisionChecker_);
        }
    };

    template <typename F, typename Signature>
    int visitTarget(const std::function<Signature> &f, visitproc visit, void *arg)
    {
        if (const auto *target = f.template target<F>())
            if (nb::handle held = target->fn.exclusive(); held.is_valid())
                Py_VISIT(held.ptr());
        return 0;
    }

    template <typename Planner>
    int visitContinuous(Planner &p, visitproc visit, void *arg)
    {
        const auto *pub = p.continuousSimulator.template target<ContinuousSimulator>();
        const auto *prot = Access<Planner>::continuousSimulator(p).template target<ContinuousSimulator>();
        for (const ContinuousSimulator *sim : {pub, prot})
        {
            if (sim == nullptr || (sim == prot && pub != nullptr && pub->fn == prot->fn))
                continue;
            const long holders =
                (pub != nullptr && pub->fn == sim->fn ? 1 : 0) + (prot != nullptr && prot->fn == sim->fn ? 1 : 0);
            if (sim->fn.use_count() == holders)
                if (nb::handle held = sim->fn->exclusive(); held.is_valid())
                    Py_VISIT(held.ptr());
        }
        return 0;
    }

    // ob.Planner has slots of its own, so chain to them rather than visiting the type twice.
    inline int traverseBase(PyObject *self, visitproc visit, void *arg)
    {
        auto *base = reinterpret_cast<PyTypeObject *>(nb::type<ob::Planner>().ptr());
        if (auto baseTraverse = reinterpret_cast<traverseproc>(PyType_GetSlot(base, Py_tp_traverse)))
            return baseTraverse(self, visit, arg);
        Py_VISIT(Py_TYPE(self));
        return 0;
    }

    inline void clearBase(PyObject *self)
    {
        auto *base = reinterpret_cast<PyTypeObject *>(nb::type<ob::Planner>().ptr());
        if (auto baseClear = reinterpret_cast<inquiry>(PyType_GetSlot(base, Py_tp_clear)))
            baseClear(self);
    }

    template <typename Planner>
    int traverse(PyObject *self, visitproc visit, void *arg)
    {
        if (int rc = traverseBase(self, visit, arg))
            return rc;
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;

        using A = Access<Planner>;
        Planner &p = *nb::inst_ptr<Planner>(self);
        for (const auto *set : {&A::jumpSet(p), &A::flowSet(p), &A::unsafeSet(p)})
            if (int rc = gc::visitHeld(*set, visit, arg))
                return rc;
        if (int rc = gc::visitHeld(A::distanceFunc(p), visit, arg))
            return rc;
        if (int rc = visitTarget<DiscreteSimulator>(A::discreteSimulator(p), visit, arg))
            return rc;
        if (int rc = visitTarget<CollisionChecker<typename A::Motion>>(A::collisionChecker(p), visit, arg))
            return rc;
        return visitContinuous(p, visit, arg);
    }

    template <typename Planner>
    int clear(PyObject *self)
    {
        if (nb::inst_ready(self) && gc::pythonOwned(self))
        {
            using A = Access<Planner>;
            Planner &p = *nb::inst_ptr<Planner>(self);
            for (auto *set : {&A::jumpSet(p), &A::flowSet(p), &A::unsafeSet(p)})
                if (gc::holdsPython(*set))
                    *set = nullptr;
            if (gc::holdsPython(A::distanceFunc(p)))
                A::distanceFunc(p) = nullptr;
            if (A::discreteSimulator(p).template target<DiscreteSimulator>() != nullptr)
                A::discreteSimulator(p) = nullptr;
            if (A::collisionChecker(p).template target<CollisionChecker<typename A::Motion>>() != nullptr)
                A::collisionChecker(p) = nullptr;
            if (A::continuousSimulator(p).template target<ContinuousSimulator>() != nullptr)
                A::continuousSimulator(p) = nullptr;
            if (p.continuousSimulator.template target<ContinuousSimulator>() != nullptr)
                p.continuousSimulator = nullptr;
        }
        clearBase(self);
        return 0;
    }

    template <typename Planner>
    PyType_Slot slots[] = {{Py_tp_traverse, (void *)traverse<Planner>}, {Py_tp_clear, (void *)clear<Planner>}, {0, 0}};

    // Motions belong to the planner and are only valid during the callback that receives them.
    template <typename Motion, typename Scope>
    nb::class_<Motion> bindMotion(Scope &scope)
    {
        return nb::class_<Motion>(scope, "Motion")
            .def_prop_ro(
                "state", [](const Motion &m) { return m.state; }, nb::rv_policy::reference)
            .def_prop_ro(
                "parent", [](const Motion &m) { return m.parent; }, nb::rv_policy::reference)
            .def_prop_ro(
                "control", [](const Motion &m) { return m.control; }, nb::rv_policy::reference)
            .def_prop_ro("solutionPair",
                         [](const Motion &m) -> nb::object
                         {
                             if (m.solutionPair == nullptr)
                                 return nb::none();
                             nb::list states;
                             for (ob::State *s : *m.solutionPair)
                                 states.append(nb::cast(s, nb::rv_policy::reference));
                             return states;
                         });
    }

    template <typename Planner>
    nb::class_<Planner, ob::Planner> bindPlanner(nb::module_ &m, const char *name)
    {
        using Motion = typename Planner::Motion;
        using A = Access<Planner>;

        nb::class_<Planner, ob::Planner> cls(m, name, nb::type_slots(slots<Planner>));
        cls.def(nb::init<const oc::SpaceInformationPtr &>(), nb::arg("si"))
            .def("solve", ompl::binding::solveBinding<Planner>(), nb::arg("what"))
            .def("solve", ompl::binding::solveFnBinding<Planner>(), nb::arg("terminationConditionFn"),
                 nb::arg("checkInterval") = 0.0)
            .def(
                "getPlannerData", [](const Planner &self, ob::PlannerData &data) { self.getPlannerData(data); },
                nb::arg("data"))
            .def("clear", &Planner::clear)
            .def("setup", &Planner::setup)
            .def("setTm", &Planner::setTm, nb::arg("tM"))
            .def("setFlowStepDuration", &Planner::setFlowStepDuration, nb::arg("duration"))
            .def("checkMandatoryParametersSet", &Planner::checkMandatoryParametersSet)
            .def(
                "setJumpSet", [](Planner &p, nb::callable fn)
                { p.setJumpSet(gc::function<typename A::SetFn>(std::move(fn))); }, nb::arg("jumpSet"))
            .def(
                "setFlowSet", [](Planner &p, nb::callable fn)
                { p.setFlowSet(gc::function<typename A::SetFn>(std::move(fn))); }, nb::arg("flowSet"))
            .def(
                "setUnsafeSet", [](Planner &p, nb::callable fn)
                { p.setUnsafeSet(gc::function<typename A::SetFn>(std::move(fn))); }, nb::arg("unsafeSet"))
            .def(
                "setDistanceFunction", [](Planner &p, nb::callable fn)
                { p.setDistanceFunction(gc::function<typename A::DistanceFn>(std::move(fn))); }, nb::arg("function"))
            .def(
                "setDiscreteSimulator",
                [](Planner &p, nb::callable fn)
                {
                    p.setDiscreteSimulator(
                        DiscreteSimulator{gc::SharedRef(std::move(fn)), p.getSpaceInformation().get()});
                },
                nb::arg("function"))
            .def(
                "setContinuousSimulator",
                [](Planner &p, nb::callable fn)
                {
                    p.continuousSimulator = ContinuousSimulator{std::make_shared<const gc::SharedRef>(std::move(fn)),
                                                                p.getSpaceInformation().get()};
                },
                nb::arg("function"))
            .def(
                "setCollisionChecker", [](Planner &p, nb::callable fn)
                { p.setCollisionChecker(CollisionChecker<Motion>{gc::SharedRef(std::move(fn))}); },
                nb::arg("function"));
        return cls;
    }
}  // namespace ompl::binding::control::hybrid
