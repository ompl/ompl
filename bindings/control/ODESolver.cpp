#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/function.h>
#include <sstream>

#include "ompl/control/ODESolver.h"
#include "ompl/control/SpaceInformation.h"
#include "init.hh"

// The default runge_kutta4 or runge_kutta_cash_karp54 are from Boost numeric/odeint
// which are used in ODEBasicSolver, ODEErrorSolver, ODEAdaptiveSolver.

namespace nb = nanobind;
namespace oc = ompl::control;
namespace ob = ompl::base;

void ompl::binding::control::init_ODESolver(nb::module_ &m)
{
    nb::class_<oc::ODESolver>(m, "ODESolver");
}
