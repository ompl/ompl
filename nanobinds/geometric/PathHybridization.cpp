#include <nanobind/nanobind.h>
#include "ompl/geometric/PathHybridization.h"
#include "init.h"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::init_PathHybridization(nb::module_ &m)
{
    nb::class_<og::PathHybridization>(m, "PathHybridization");
}
