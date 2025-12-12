#include <nanobind/nanobind.h>
#include "ompl/geometric/planners/informedtrees/bitstar/CostHelper.h"
#include "../../../init.hh"

namespace nb = nanobind;
namespace ob = ompl::base;
namespace og = ompl::geometric;

void ompl::binding::geometric::initPlannersInformedtreesBitstar_CostHelper(nb::module_& m)
{
    // TODO [og::BITstar::CostHelper][IMPLEMENT]
    nb::class_<og::BITstar::CostHelper>(m, "CostHelper")
        ;
}
