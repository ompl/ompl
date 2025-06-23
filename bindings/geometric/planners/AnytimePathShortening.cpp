#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/AnytimePathShortening.h"
#include "../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlanners_AnytimePathShortening(nb::module_& m)
{
    // TODO [og::AnytimePathShortening][IMPLEMENT]
    nb::class_<og::AnytimePathShortening, ob::Planner>(m, "AnytimePathShortening")
        ;
}
