#include <nanobind/nanobind.h>
#include "ompl/geometric/PathHybridization.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::init_PathHybridization(nb::module_& m)
{
    // TODO [og::PathHybridization][IMPLEMENT]
    nb::class_<og::PathHybridization>(m, "PathHybridization")
        ;
}
