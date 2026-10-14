#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/control/ODESolver.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/spaces/DiscreteControlSpace.h"
#include "ompl/util/Exception.h"
#include "PyGC.h"
#include "init.h"

namespace nb = nanobind;
namespace gc = ompl::binding::gc;
namespace oc = ompl::control;
namespace ob = ompl::base;

namespace
{
    struct ODEWrapper
    {
        nb::callable fn;

        explicit ODEWrapper(nb::callable fn) : fn(std::move(fn))
        {
        }
    };

    nb::list controlToList(const oc::SpaceInformationPtr &si, const oc::Control *u)
    {
        nb::list result;
        const auto &cspace = si->getControlSpace();
        const unsigned dim = cspace->getDimension();

        if (dynamic_cast<const oc::RealVectorControlSpace *>(cspace.get()) != nullptr)
        {
            const auto *ctrl = static_cast<const oc::RealVectorControlSpace::ControlType *>(u);
            for (unsigned i = 0; i < dim; ++i)
                result.append((*ctrl)[i]);
        }
        else if (dynamic_cast<const oc::DiscreteControlSpace *>(cspace.get()) != nullptr)
        {
            const auto *ctrl = static_cast<const oc::DiscreteControlSpace::ControlType *>(u);
            result.append(ctrl->value);
        }
        else
        {
            auto *mutableCspace = const_cast<oc::ControlSpace *>(cspace.get());
            auto *mutableControl = const_cast<oc::Control *>(u);
            for (unsigned i = 0; i < dim; ++i)
            {
                if (double *val = mutableCspace->getValueAddressAtIndex(mutableControl, i))
                    result.append(*val);
                else
                    result.append(0.0);
            }
        }
        return result;
    }

    void callODE(nb::handle fn, const oc::SpaceInformationPtr &si, const oc::ODESolver::StateType &x,
                 const oc::Control *u, oc::ODESolver::StateType &xnew)
    {
        nb::gil_scoped_acquire gil;
        // nanobind's std::vector caster converts by copy, so the derivative has to be handed to Python as a
        // list and read back out; passing xnew directly would silently discard everything the ODE writes.
        nb::list qdot;
        for (std::size_t i = 0; i < x.size(); ++i)
            qdot.append(0.0);
        fn(x, controlToList(si, u), qdot);
        xnew.resize(x.size());
        for (std::size_t i = 0; i < x.size(); ++i)
            xnew[i] = nb::cast<double>(qdot[i]);
    }

    nb::object resolveODE(nb::object ode)
    {
        if (nb::isinstance<ODEWrapper>(ode))
            return nb::cast<ODEWrapper &>(ode).fn;
        if (!nb::isinstance<nb::callable>(ode))
            throw nb::type_error("ode must be callable or an ompl.control.ODE");
        return ode;
    }

    /// Holds the Python ODE in a member rather than inside the std::function stored in ODESolver::ode_.
    /// A callable captured by that std::function is invisible to the collector, so an ODE that reaches its
    /// own solver -- a bound method, or a closure over the SpaceInformation -- could never be collected.
    template <typename Solver>
    struct PyODESolver : Solver
    {
        nb::object ode;

        PyODESolver(const oc::SpaceInformationPtr &si, nb::object fn, double intStep)
          : Solver(
                si, [this](const oc::ODESolver::StateType &x, const oc::Control *u, oc::ODESolver::StateType &xnew)
                { callODE(ode, this->getSpaceInformation(), x, u, xnew); }, intStep)
          , ode(std::move(fn))
        {
        }

        // The stored ODE captures `this`.
        PyODESolver(const PyODESolver &) = delete;
        PyODESolver &operator=(const PyODESolver &) = delete;
    };

    using BasicSolver = PyODESolver<oc::ODEBasicSolver<>>;
    using ErrorSolver = PyODESolver<oc::ODEErrorSolver<>>;

    /// ODESolver::si_ was built by nanobind's shared_ptr caster, which keeps the Python SpaceInformation alive
    /// with a reference the collector cannot see. Reporting it here balances that reference, so a solver
    /// reachable from its own SpaceInformation does not pin the pair forever.
    template <typename Solver>
    int solverTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;

        const auto *solver = nb::inst_ptr<Solver>(self);
        Py_VISIT(solver->ode.ptr());
        return gc::visitHeld(solver->getSpaceInformation(), visit, arg);
    }

    template <typename Solver>
    int solverClear(PyObject *self)
    {
        if (nb::inst_ready(self) && gc::pythonOwned(self))
            nb::inst_ptr<Solver>(self)->ode.reset();
        return 0;
    }

    template <typename Solver>
    PyType_Slot solverSlots[] = {{Py_tp_traverse, (void *)solverTraverse<Solver>},
                                 {Py_tp_clear, (void *)solverClear<Solver>},
                                 {0, 0}};

    /// ODESolver::getStatePropagator() hides its propagator in a private nested class, so wrap it. Owning the
    /// Python solver here keeps the C++ solver alive for as long as the propagator can be used -- upstream only
    /// takes a non-owning alias -- and puts that reference where traverse and clear can reach it.
    struct ODEStatePropagator : oc::StatePropagator
    {
        nb::object solver;
        nb::object postEvent;
        oc::StatePropagatorPtr impl;

        ODEStatePropagator(oc::ODESolver *raw, nb::object solverObj, nb::object postEventObj)
          : oc::StatePropagator(raw->getSpaceInformation())
          , solver(std::move(solverObj))
          , postEvent(std::move(postEventObj))
          , impl(oc::ODESolver::getStatePropagator(oc::ODESolverPtr(raw, [](oc::ODESolver *) {})))
        {
        }

        void propagate(const ob::State *state, const oc::Control *control, double duration,
                       ob::State *result) const override
        {
            if (!impl)
                throw ompl::Exception("ODE state propagator was cleared by Python's garbage collector");

            impl->propagate(state, control, duration, result);
            if (postEvent.is_valid())
            {
                nb::gil_scoped_acquire gil;
                postEvent(state, control, duration, result);
            }
        }
    };

    int propagatorTraverse(PyObject *self, visitproc visit, void *arg)
    {
        Py_VISIT(Py_TYPE(self));
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;

        const auto *prop = nb::inst_ptr<ODEStatePropagator>(self);
        Py_VISIT(prop->solver.ptr());
        Py_VISIT(prop->postEvent.ptr());
        return 0;
    }

    int propagatorClear(PyObject *self)
    {
        if (!nb::inst_ready(self) || !gc::pythonOwned(self))
            return 0;

        auto *prop = nb::inst_ptr<ODEStatePropagator>(self);
        // Drops the alias to the solver along with the reference keeping the solver alive.
        prop->impl.reset();
        prop->solver.reset();
        prop->postEvent.reset();
        return 0;
    }

    PyType_Slot propagatorSlots[] = {{Py_tp_traverse, (void *)propagatorTraverse},
                                     {Py_tp_clear, (void *)propagatorClear},
                                     {0, 0}};

    template <typename Solver>
    nb::class_<Solver, oc::ODESolver> bindSolver(nb::module_ &m, const char *name)
    {
        return nb::class_<Solver, oc::ODESolver>(m, name, nb::type_slots(solverSlots<Solver>))
            .def(
                "__init__", [](Solver *self, const oc::SpaceInformationPtr &si, nb::object ode, double intStep)
                { new (self) Solver(si, resolveODE(std::move(ode)), intStep); }, nb::arg("si"), nb::arg("ode"),
                nb::arg("intStep") = 1e-2)
            .def(
                "setODE", [](Solver &solver, nb::object ode) { solver.ode = resolveODE(std::move(ode)); },
                nb::arg("ode"));
    }
}  // namespace

void ompl::binding::control::init_ODESolver(nb::module_ &m)
{
    nb::class_<ODEWrapper>(m, "ODE").def(nb::init<nb::callable>(), nb::arg("fn"));

    nb::class_<ODEStatePropagator, oc::StatePropagator>(m, "ODEStatePropagator", nb::type_slots(propagatorSlots));

    nb::class_<oc::ODESolver>(m, "ODESolver")
        .def("getIntegrationStepSize", &oc::ODESolver::getIntegrationStepSize)
        .def("setIntegrationStepSize", &oc::ODESolver::setIntegrationStepSize, nb::arg("intStep"))
        .def("getSpaceInformation", &oc::ODESolver::getSpaceInformation, nb::rv_policy::reference_internal)
        .def_static(
            "getStatePropagator",
            [](nb::object solver, nb::object postEvent)
            {
                auto *raw = nb::cast<oc::ODESolver *>(solver);
                if (raw == nullptr)
                    throw nb::type_error("solver must be an ompl.control.ODESolver");
                if (!postEvent.is_none())
                    postEvent = nb::cast<nb::callable>(postEvent);
                else
                    postEvent = nb::object();
                // Python owns the propagator, so its traverse may report the solver and postEvent it keeps alive.
                auto prop = std::make_unique<ODEStatePropagator>(raw, std::move(solver), std::move(postEvent));
                nb::object result = nb::cast(prop.get(), nb::rv_policy::take_ownership);
                prop.release();
                return result;
            },
            nb::arg("solver"), nb::arg("postEvent") = nb::none());

    bindSolver<BasicSolver>(m, "ODEBasicSolver");
    bindSolver<ErrorSolver>(m, "ODEErrorSolver").def("getError", &ErrorSolver::getError);
}
