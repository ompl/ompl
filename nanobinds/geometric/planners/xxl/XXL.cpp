#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/xxl/XXL.h"
#include "../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersXxl_XXL(nb::module_& m)
{
    // TODO [og::XXL][IMPLEMENT]
    // TAG [og::XXL][Planner]
    nb::class_<og::XXL, ob::Planner>(m, "XXL")
        ;
}
