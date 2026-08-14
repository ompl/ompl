#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/vector.h>

#include "ompl/control/ODESolver.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/control/spaces/DiscreteControlSpace.h"
#include "init.h"

namespace nb = nanobind;
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

        if (const auto *rv = dynamic_cast<const oc::RealVectorControlSpace *>(cspace.get()))
        {
            const auto *ctrl = static_cast<const oc::RealVectorControlSpace::ControlType *>(u);
            for (unsigned i = 0; i < dim; ++i)
                result.append((*ctrl)[i]);
        }
        else if (const auto *dc = dynamic_cast<const oc::DiscreteControlSpace *>(cspace.get()))
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

    oc::ODESolver::ODE wrapPythonODE(const oc::SpaceInformationPtr &si, nb::callable fn)
    {
        return [si, fn = nb::callable(fn)](const oc::ODESolver::StateType &x, const oc::Control *u,
                                           oc::ODESolver::StateType &xnew)
        {
            nb::list ulist = controlToList(si, u);
            fn(x, ulist, xnew);
        };
    }

    oc::ODESolver::ODE resolveODE(const oc::SpaceInformationPtr &si, nb::object ode)
    {
        if (nb::isinstance<ODEWrapper>(ode))
            return wrapPythonODE(si, nb::cast<ODEWrapper>(ode).fn);
        if (nb::isinstance<nb::callable>(ode))
            return wrapPythonODE(si, nb::cast<nb::callable>(ode));
        return nb::cast<oc::ODESolver::ODE>(ode);
    }

    oc::ODESolver::PostPropagationEvent wrapPythonPostEvent(nb::callable fn)
    {
        return [fn = nb::callable(fn)](const ob::State *state, const oc::Control *control, double duration,
                                       ob::State *result) { fn(state, control, duration, result); };
    }
}  // namespace

using BasicSolver = oc::ODEBasicSolver<>;
using ErrorSolver = oc::ODEErrorSolver<>;

void ompl::binding::control::init_ODESolver(nb::module_ &m)
{
    nb::class_<ODEWrapper>(m, "ODE").def(nb::init<nb::callable>(), nb::arg("fn"));

    nb::class_<oc::ODESolver>(m, "ODESolver")
        .def("setODE", &oc::ODESolver::setODE, nb::arg("ode"))
        .def("getIntegrationStepSize", &oc::ODESolver::getIntegrationStepSize)
        .def("setIntegrationStepSize", &oc::ODESolver::setIntegrationStepSize, nb::arg("intStep"))
        .def("getSpaceInformation", &oc::ODESolver::getSpaceInformation, nb::rv_policy::reference_internal)
        .def_static(
            "getStatePropagator",
            [](oc::ODESolver &solver, nb::object postEvent)
            {
                const oc::ODESolverPtr ptr(&solver, [](oc::ODESolver *) {});
                if (postEvent.is_none())
                    return oc::ODESolver::getStatePropagator(ptr);
                return oc::ODESolver::getStatePropagator(ptr, wrapPythonPostEvent(nb::cast<nb::callable>(postEvent)));
            },
            nb::arg("solver"), nb::arg("postEvent") = nb::none());

    nb::class_<BasicSolver, oc::ODESolver>(m, "ODEBasicSolver")
        .def(
            "__init__", [](BasicSolver *self, const oc::SpaceInformationPtr &si, nb::object ode, double intStep)
            { new (self) BasicSolver(si, resolveODE(si, ode), intStep); }, nb::arg("si"), nb::arg("ode"),
            nb::arg("intStep") = 1e-2);

    nb::class_<ErrorSolver, oc::ODESolver>(m, "ODEErrorSolver")
        .def(
            "__init__", [](ErrorSolver *self, const oc::SpaceInformationPtr &si, nb::object ode, double intStep)
            { new (self) ErrorSolver(si, resolveODE(si, ode), intStep); }, nb::arg("si"), nb::arg("ode"),
            nb::arg("intStep") = 1e-2)
        .def("getError", &ErrorSolver::getError);
}
