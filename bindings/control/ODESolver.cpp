#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <nanobind/trampoline.h>
#include <sstream>

#include "ompl/control/ODESolver.h"
#include "ompl/control/SpaceInformation.h"
#include "init.hh"

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_ODESolver(nb::module_ &m)
{
    // struct PyODESolver : oc::ODESolver
    // {
    //     NB_TRAMPOLINE(oc::ODESolver, 1);
    //     void solve(oc::ODESolver::StateType &state, oc::Control *control, double duration) const override
    //     {
    //         NB_OVERRIDE_PURE(solve, state, control, duration);
    //     }
    // };
    
    // TODO [oc::ODESolver][IMPLEMENT]
    nb::class_<oc::ODESolver>(m, "ODESolver");
        // .def(nb::init<oc::SpaceInformationPtr, oc::ODESolver::ODE, double>(),
        //      nb::arg("si"), nb::arg("ode"), nb::arg("intStep"));
}
