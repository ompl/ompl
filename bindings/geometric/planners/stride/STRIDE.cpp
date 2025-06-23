#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/stride/STRIDE.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersStride_STRIDE(nb::module_& m)
{
    // TODO [og::STRIDE][IMPLEMENT]
    // TAG [og::STRIDE][Planner]
    nb::class_<og::STRIDE, ob::Planner>(m, "STRIDE")
        ;
}
