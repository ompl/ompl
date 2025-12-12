#include <nanobind/nanobind.h>
#include "ompl/geometric/HillClimbing.h"
#include "init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::init_HillClimbing(nb::module_& m)
{
    // TODO [og::HillClimbing][IMPLEMENT]
    nb::class_<og::HillClimbing>(m, "HillClimbing")
        ;
}
